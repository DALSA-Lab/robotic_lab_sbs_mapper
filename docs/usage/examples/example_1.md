# Example: Instantiating a CalibratedCamera instance

This example demonstrates how to instantiate a CalibratedCamera object. 

Three methods for creating a new CalibratedCamera object exist. Each method is covered in its own section below. 

## Perform a calibration
To perform a calibration, a handful of prerequisites must be met, namely:
1. A list of robot poses (X, Y, Z) and rotation matrix must be provided
2. Collection of images, one for each pose, must be provided
3. Meassured calibration chessboard dimensions

The list of robot poses and collection of images must follow the same order. To ensure the correct ordering, it is advised to name the images during image capture to follow the robot pose index. 

The chessboard dimensions include the physical size of a single square in millimeters, and the number of inner corners column and row wise.

For this example, the robot poses are stored in an array following the structure from the [ur_commander](https://github.com/DALSA-Lab/ur_commander) `TaskPose`. Each TaskPose is made up of an XYZ coordinate and RxRyRz rotation vector (axis-angle). The rotation vectors must be converted to rotation matrixes before calibration
```python
# imports
import cv2 as cv
import numpy as np
import glob # needed to load images
import re # regular expression import needed to sort images
from mapit import CalibratedCamera

# list of robot poses
poses = [{
        "tool_pose": [
            -0.2918351813985059,
            -0.20152920835988364,
            0.12129173093816012,
            0.06401085239746532,
            -3.0377776858357786,
            0.6808930384830819
        ]
    },
    ...
    {
        "tool_pose": [
            -0.2194709077852959,
            -0.08776464280537256,
            0.06281433261243191,
            2.215524035649932,
            -2.0041022662691965,
            0.38228743199893733
        ]
    }
]
# arrays to store rotation matrixes and translation vectors
R_gripper2base = []
t_gripper2base = []
for pose in poses:
    if pose["tool_pose"] != None:
        tool_pose = pose["tool_pose"]
        x, y, z = tool_pose[0:3]
        rx, ry, rz = tool_pose[3:6]

        rvec = np.array([rx,ry,rz], dtype=np.float64)

        R, _ = cv2.Rodrigues(rvec)

        R_gripper2base.append(R)
        t_gripper2base.append(np.array([x,y,z], dtype=np.float64))
```
Next, the images of the calibration board are loaded. The filenames follow the convention of `image_waypointXXX.jpg` with XXX representing the pose index.
```python
# load images from folder
source_folder = "/home/jesper/Pictures/" 
# select all files matching the .jpg extension
images_loc = glob.glob(source_folder + "*.jpg")
# sort images based on their filename
images_loc = sorted(images_loc, key=lambda x: int(re.findall(r'\d+', x)[-1]))
images = []
for fname in images_loc:
    image = cv.imread(fname)
    images.append(image)
```

The last step before calibration is defining the dimensions of the calibration board. This is done by counting the internal corners, and by meassuring the side length of a single square, reported in meters.
```python
board_height = 5
board_width = 8
square_size_m = 0.03 # 30mm square side length 
board = (board_height, board_width, square_size_m)
```

A new `CalibratedCamera` object can be obtained by performing a new calibration.
```python
# create new CalibratedCamera object
camera = CalibratedCamera()
camera.new_calibration(images, R_gripper2base, t_gripper2base, board)
```


## Load a calibration from file
To create a new object from a previously calibrated instance, use the `new_from_config` function:
```python
# imports
from mapit import CalibratedCamera

# create new CalibratedCamera object
camera = CalibratedCamera.new_from_config("calibrated_camera.yml", None)

# to export a CalibratedCamera object, use the export_to_config function
camera.export_to_config("calibrated_camera2.yml")
```
## Manual instantiation
To manually instantiate a CalibratedCamera object, camera parameters must be provided.

```python
import numpy as np
from mapit import CalibratedCamera

R_cam2tcp = np.array([
    [0.7031712873773839, -0.7108982951179903, -0.01318160105478331],
    [0.7110113851610904, 0.7029476017317904, 0.01809639165225672],
    [-0.003598719124931573, -0.02209713143960507, 0.9997493515890874]
], dtype=np.float64)

T_cam2tcp = np.array( [0.01239759056065678,
 -0.0570573497099558,
 0.01802776294011691], dtype=np.float64
)
camera_intrinsics = np.array([
    [ 1364.83618164062, 0.0, 971.963623046875],
    [0.0, 1364.55151367188, 575.387329101562],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

distortion_coefficients = np.array([
    0.0, 0.0, 0.0, 0.0, 0.0
], dtype=np.float64)


camera = CalibratedCamera(
    R_cam2tcp=R_cam2tcp,
    T_cam2tcp=T_cam2tcp,
    distortion_coefficients=distortion_coefficients,
    intrinsics=camera_intrinsics,
    frame_grapper=None,
    image_height=1080,  # pixels
    image_width=1920,   # pixels
)
```