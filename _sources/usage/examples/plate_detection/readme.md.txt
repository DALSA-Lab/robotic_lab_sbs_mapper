# Dynamic Plate Detection and Localisation
## Overview

This example demonstrates how to dynamically detect, localise, and retrieve an SBS plate in a changing laboratory environment using ArUco markers and a calibrated eye-in-hand camera setup.

The key idea is to:

1. Perform an initial scan of the workspace from multiple robot poses.
2. Detect ArUco markers attached to lab objects (e.g. a Mantis device and an SBS plate).
3. Estimate each marker pose in the robot base frame.
4. Refine these poses through active visual alignment.
5. Compute spatial relationships between objects (e.g. plate relative to Mantis).
6. Re-scan after the environment changes.
7. Reconstruct the missing plate pose from known spatial relationships.
8. Execute a pick operation using the reconstructed pose.

This workflow enables robust re-localisation of objects even when only a subset of markers is visible during a later scan.

The example uses:

- A calibrated RGB camera (Intel RealSense).
- A UR robot controlled via ur_commander.
- The mapit package for camera calibration, ArUco detection, pose estimation, and frame transformations.

A video demonstration is available on [YouTube](https://youtu.be/aCD06wiFVYI).

## Physical Setup

Before running this example, ensure that the following hardware and configuration steps are completed.

### 1. Robot and Camera

The system uses a UR robot equipped with a TCP-mounted RGB camera in an eye-in-hand configuration.

The following prerequisites must be satisfied:

- A valid hand–eye calibration stored in `calibrated_camera.yml`.
- A known rigid transformation between the camera and the TCP, defined by `R_cam2tcp` and `T_cam2tcp`.

These parameters allow detected marker poses to be expressed consistently in the robot base frame.

### 2. Marked Objects

All relevant objects in the workspace must carry an ArUco marker.

In this example, the following marker IDs are used:

```python
mantis_id = 5
plate_id = 8
```

Markers must be generated from the same dictionary used by the detector.  
Here, the `cv.aruco.DICT_4X4_50` dictionary is used.

### 3. Scanning Poses

A set of predefined joint configurations (`scanning_poses`) must be provided.

These poses should:

- Cover the relevant workspace area.
- Ensure marker visibility from multiple viewpoints.
- Minimise occlusions.
- Maintain safe robot motion.

Careful selection of scanning poses improves robustness and pose estimation accuracy.

---

## Initialisation

The system initialises the perception and robot-control pipeline in several steps.

```python
# imports
import logging

import cv2 as cv
import numpy as np
import pyrealsense2 as rs
from ur_commander import CustomURRobot, JointPositions, TaskPose

from mapit import CalibratedCamera, Detector, Marker
from mapit.utils import (
    apply_transformation,
    build_pose,
    extract_pose_and_orientation,
    make_homogeneous,
    rot_tran_from_tool_pose,
)

```

### 1. RealSense Camera

A color stream from the RealSense camera is started and warmed up with the following configuration:

- Resolution: **1920 × 1080**
- Format: **BGR8**
- Frame rate: **30 FPS**

A frame grabber function is then registered with `CalibratedCamera`, enabling the detector to retrieve live images directly from the camera stream.

```python
# init realsense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
profile = pipeline.start(config)
# Warm up
for _ in range(30):
    pipeline.wait_for_frames()


# define function for grabbing a frame.
# if window is specified, show the grabbed frame
def get_realsense_frame(pipeline, window=None):
    rawframes = pipeline.wait_for_frames()
    raw_color_frame = rawframes.get_color_frame()
    img = np.asanyarray(raw_color_frame.get_data())
    if window:
        cv.imshow(window, img)
        cv.waitKey(1)  # non-zero wait to allow image render
    return img


CV_NAMED_WINDOW = "feed"
cv.namedWindow(CV_NAMED_WINDOW)
```
A `namedWindow` is specified to be able to follow the video feed.

### 2. Loading a `CalibratedCamera` config



### 3. Robot Interface

A `CustomURRobot` instance is created to communicate with the UR robot controller.

During initialisation:

- URScript programs for opening and closing the gripper are loaded.
- The gripper is opened to ensure a safe starting configuration before scanning begins.

### 4. ArUco Detector

A `Detector` instance is created using:

- The `DICT_4X4_50` ArUco dictionary.
- The `CalibratedCamera` instance loaded above.
- Built-in pose estimation utilities.

This detector is responsible for identifying markers in each frame and estimating their 6D pose relative to the camera.
## Performing a Scan

The `scan()` routine performs the initial localisation of all visible markers in the workspace.  
This step combines robot motion, ArUco detection, pose estimation, and frame transformations to express each marker in the robot base frame.

Rather than showing the entire implementation, the key `mapit` interactions are highlighted below.

---

### 1. Moving to Predefined Scanning Poses

The robot iterates through a set of predefined joint configurations to observe the workspace from multiple viewpoints:

```python
robot.movej(JointPositions(joint_pose))
```

Well-chosen scanning poses improve visibility, reduce occlusions, and increase pose accuracy.

---

### 2. Detecting ArUco Markers

At each pose, the detector processes the current camera frame:

```python
detected_markers = d.detect_markers()
```

The `Detector` instance internally:

- Retrieves the latest frame from `CalibratedCamera`
- Detects ArUco markers
- Returns image-space corner observations grouped by marker ID

If no markers are detected, the scan continues to the next robot pose.

---

### 3. Estimating Marker Pose in the Camera Frame

For each detected marker, a 6D pose is estimated relative to the camera:

```python
H_marker_in_cam, ok = d.estimate_marker_pose(img_points)
```

If successful, this returns a homogeneous transformation matrix. This is one of the core capabilities provided by `mapit`: converting raw image detections into metric 3D poses using camera calibration.

---

### 4. Transforming to the Robot Base Frame

The marker pose must be expressed in the robot base coordinate system.  
This is done in two steps using `mapit.utils`:

```python
marker_in_tcp = apply_transformation(H_marker_in_cam, H_cam2tcp)
marker_in_base = apply_transformation(marker_in_tcp, H_tcp2base)
```

Where:

- `H_cam2tcp` — fixed hand–eye calibration
- `H_tcp2base` — current robot forward kinematics

The helper function `apply_transformation` ensures consistent and readable frame transformations.

---

### 5. Storing the Detected Objects

Each detected marker is stored together with the robot pose at detection time:

```python
detected_ids[id] = {
    "M": Marker(id=id, H=marker_in_base),
    "robot": tool_pose
}
```

The `Marker` object encapsulates:

- Marker ID
- Full homogeneous transform in the base frame
- Convenient access to position and orientation

---

### Scan Result

The `scan()` function returns a dictionary:

```python
detected_ids = scan()
```

Each entry contains a marker expressed in the robot base frame.  
This representation provides a consistent geometric foundation for:

- Pose refinement  
- Spatial relationship computation  
- Object re-localisation  
- Manipulation planning  

At this stage, all visible objects have an initial 6D pose estimate in the shared base coordinate system.

## Computing Spatial Relationships

After the initial scan, we refine each detected marker pose using the `refine()` routine.

### Active Visual Refinement

For each marker:

1. Rotate the camera to align its optical axis with the marker center.  
2. Move closer to reduce reprojection noise.  
3. Re-estimate the marker pose multiple times (200 samples).  
4. Compute:
   - Median translation (robust against outliers)  
   - Mean rotation vector (approximation)  

This produces a significantly more stable estimate of:

```
H_marker_in_base
```

The refined transform replaces the initial estimate in `detected_ids`.

---

### Computing Plate–Mantis Relationship

Once both Mantis and plate are refined:

```python
H_plate_in_mantis = H_plate_in_base @ np.linalg.inv(H_mantis_in_base)
```

This encodes the spatial relationship:

> “Where is the plate relative to the Mantis?”

This relationship is assumed rigid and constant.

---

## Re-discovery and Re-construction of Spatial Layout

Now we simulate a dynamic workspace:

The user moves the Mantis (and the plate moves with it).  
The system no longer sees the plate during the next scan.

### Step 1: Perform a New Scan

```python
detected_ids = scan()
refine(detected_ids)
```

Assume only the Mantis marker is detected.

### Step 2: Reconstruct Plate Pose

We compute the new plate pose using the stored relationship:

```python
H_plate_in_base_new = H_mantis_in_base_new @ H_plate_in_mantis
```

Even without seeing the plate, its location is reconstructed from:

- The newly detected Mantis pose  
- The previously learned rigid transform  

This enables implicit plate localisation.

---

## Plate Alignment and Pick

Once the reconstructed plate pose is available:

1. Move above the plate.  
2. Align TCP orientation with the long side of the SBS plate.  
3. Open gripper.  
4. Descend.  
5. Close gripper.  
6. Lift plate.  

The Z-axis alignment is computed by comparing:

- TCP X-axis in base frame  
- Plate X-axis in base frame  

A corrective rotation around Z is applied to ensure parallel alignment before grasping.