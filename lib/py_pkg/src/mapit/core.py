import cv2 as cv
import numpy as np
from ur_commander import CustomURRobot, TaskPose, JointPositions
from mapit import Detector
from mapit.utils import *
from mapit import CalibratedCamera
from mapit import Marker
import logging
import pyrealsense2 as rs
import glob
import re
import json
# Scanning poses for initial scan. Choose one
# own table
# scanning_poses = [ { "joint_positions": [ 2.201622486114502, -2.140475412408346, 1.351577107106344, -0.7900988024524231, -1.5704696814166468, -0.9546028693490705 ], "tool_pose": [ 0.196397209789177, -0.04195006632687614, 0.45159636056021635, 2.19754701306036, 2.232853188362549, -0.00016905713145977828 ] } ]

# right table
scanning_poses = [ { "joint_positions": [ 0.4508141875267029, -2.3580476246275843, 1.60414964357485, -1.107905702, -1.0898402372943323, -1.0404872020059308 ], "tool_pose": [ -0.02358961302646431, -0.3392187648344618, 0.46441765778047783, 2.584609968418257, -0.00918993852448859, -0.035130947322033555 ] } ]

# top + right
scanning_poses = [ { "joint_positions": [ 2.0215814113616943, -2.3580357036986292, 1.6040666739093226, -1.1078809064677735, -1.089827839528219, -1.0404980818377894 ], "tool_pose": [ 0.3365807220118142, -0.023436699325547615, 0.468678415491101, -1.9088990761918114, 1.9225138476567623, -0.5735693543765116 ] }, { "joint_positions": [ 0.4508020877838135, -2.358047147790426, 1.6040781180011194, -1.1079174441150208, -1.0898602644549769, -1.0404780546771448 ], "tool_pose": [ -0.023442205755523776, -0.33657555977862413, 0.46867671390885335, -0.010642740577652462, -2.9954775886052403, 0.8561700796562706 ] } ]

# all
scanning_poses = [ { "joint_positions": [ 2.0215814113616943, -2.3580357036986292, 1.6040666739093226, -1.1078809064677735, -1.089827839528219, -1.0404980818377894 ], "tool_pose": [ 0.3365807220118142, -0.023436699325547615, 0.468678415491101, -1.9088990761918114, 1.9225138476567623, -0.5735693543765116 ] }, { "joint_positions": [ 0.4508020877838135, -2.358047147790426, 1.6040781180011194, -1.1079174441150208, -1.0898602644549769, -1.0404780546771448 ], "tool_pose": [ -0.023442205755523776, -0.33657555977862413, 0.46867671390885335, -0.010642740577652462, -2.9954775886052403, 0.8561700796562706 ] }, { "joint_positions": [ -1.119997803364889, -2.3580318890013636, 1.6040695349322718, -1.1079337757876893, -1.0898845831500452, -1.0405295530902308 ], "tool_pose": [ -0.3365731771025343, 0.023455665594063646, 0.46867745644299885, -1.9034143192360555, -1.8900248367608354, 0.5162820516738351 ] } ]

# right + bottom
scanning_poses = [ { "joint_positions": [ 0.4508020877838135, -2.358047147790426, 1.6040781180011194, -1.1079174441150208, -1.0898602644549769, -1.0404780546771448 ], "tool_pose": [ -0.023442205755523776, -0.33657555977862413, 0.46867671390885335, -0.010642740577652462, -2.9954775886052403, 0.8561700796562706 ] }, { "joint_positions": [ -1.119997803364889, -2.3580318890013636, 1.6040695349322718, -1.1079337757876893, -1.0898845831500452, -1.0405295530902308 ], "tool_pose": [ -0.3365731771025343, 0.023455665594063646, 0.46867745644299885, -1.9034143192360555, -1.8900248367608354, 0.5162820516738351 ] } ]

# init realsense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
profile = pipeline.start(config)
#Warm up
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
        cv.waitKey(1) # non-zero wait to allow image render
    return img

CV_NAMED_WINDOW = "feed"
cv.namedWindow(CV_NAMED_WINDOW)

camera = CalibratedCamera.new_from_config("calibrated_camera.yml", None)
camera.set_frame_grabber(lambda: get_realsense_frame(pipeline, CV_NAMED_WINDOW))

H_cam2tcp = make_homogeneous(camera.R_cam2tcp, camera.T_cam2tcp)

robot = CustomURRobot("192.168.1.102", logging.INFO)

# load UR programs to open/close gripper
close_gripper = ""
open_gripper = ""

with open("/home/jesper/DTU/KAND/robotic_lab_sbs_mapper/lib/py_pkg/src/mapit/close.script") as f:
    close_gripper = f.read()

with open("/home/jesper/DTU/KAND/robotic_lab_sbs_mapper/lib/py_pkg/src/mapit/open.script") as f:
    open_gripper = f.read()
robot.send_script(script=open_gripper, blocking=True)

d = Detector(aruco_dict=cv.aruco.DICT_4X4_50, aruco_params=None, camera=camera)

mantis_id = 5
plate_id = 8


def scan():
    detected_ids: dict[int, Marker] = {}
    for poses in scanning_poses:
        tool_pose = poses["tool_pose"]
        joint_pose = poses["joint_positions"]
        robot.movej(JointPositions(joint_pose), blocking=True)
        
        # look for aruco markers in the image
        detected_markers = d.detect_markers()
        
        # if no corners were found we did not find any markers so we continue to next pose
        if not detected_markers:
            continue
        
        print("Found ids:", detected_markers.keys())
        
        # poll actual tool pose from UR
        _, tool_pose = robot.read_joint_and_task_space_data()
        # create rotation and translation from pose
        R_tcp2base, T_tcp2base = rot_tran_from_tool_pose(tool_pose)
        H_tcp2base = make_homogeneous(R_tcp2base, T_tcp2base)
        
        # for each id, compute an initial pose
        for id, img_points in detected_markers.items():
            print("target ID:", id) 
            
            H_marker_in_cam, ok = d.estimate_marker_pose(img_points)
            if not ok:
                print("failed to compute pose for marker with ID:", id)
            
            print(f"target ID: {id}'s distance from camera:\t{H_marker_in_cam[0:3, 3]}")
            
            # transform from camera to TCP
            marker_in_tcp = apply_transformation(H_marker_in_cam, H_cam2tcp)
            print(f"target ID: {id} located at (TCP space):\t{marker_in_tcp}")
            
            # transform from TCP to base            
            marker_in_base = apply_transformation(marker_in_tcp, H_tcp2base)
            print(f"target ID: {id} located at (BASE space):\t{marker_in_base}")
            
            # store for later use
            obj = {"M": Marker(id=id, H=marker_in_base), "robot": tool_pose}
            detected_ids[id] = obj
    return detected_ids


def refine(detected_ids):
    # refinement of pose for each marker
    for id in detected_ids:
        obj = detected_ids[id]
        marker = obj["M"]
        
        tool_pose = obj["robot"]
        
        R_tcp2base, T_tcp2base = rot_tran_from_tool_pose(tool_pose)
        H_tcp2base = make_homogeneous(R_tcp2base, T_tcp2base)   
                    
        # compute the required tool rotation to align camera to marker
        origin = apply_transformation(camera.T_cam2tcp, H_tcp2base)
        desired_rotvec = d.align_camera_to_point(origin=origin, target=marker.position.reshape(3,), R_tcp2base=R_tcp2base, T_tcp2base=T_tcp2base.reshape(3,))
        
        # align camera to marker center at scanning pose
        tool_pose = build_pose(T_tcp2base, desired_rotvec)    
        robot.movej(TaskPose(tool_pose), blocking=True)
        
        joint_positions, tool_pose = robot.read_joint_and_task_space_data()
        R_tcp2base, T_tcp2base = rot_tran_from_tool_pose(tool_pose)
        H_tcp2base = make_homogeneous(R_tcp2base, T_tcp2base)

        distance = np.sqrt(np.power(marker.position[0] - T_tcp2base[0],2)+np.power(marker.position[1] - T_tcp2base[1],2)+np.power(marker.position[2] - T_tcp2base[2],2))
        z = distance - 0.25
        v = np.array([[0],[0],[1]])
        v = (v / np.linalg.norm(v) ) * z
        v_in_base = apply_transformation(v.flatten(), H_tcp2base)
        
        new_pose = build_pose(v_in_base, desired_rotvec)
        
        robot.movej(TaskPose(new_pose), blocking=True, a=0.5)
        
        # look for aruco markers in the image
        detected_markers = d.detect_markers()
        
        # if no markers were found we continue to next pose
        if not detected_markers:
            continue
        
        # check if the targeted marker was one of the detected markers
        if not id in detected_markers:
            print("unable to find target dict in frame")
            break

        # estimate the pose again
        H_marker_in_cam, ok = d.estimate_marker_pose(detected_markers[id])
        if not ok:
            print("failed to compute pose for marker with ID:", id)
            continue
        
        joint_positions, tool_pose = robot.read_joint_and_task_space_data()
        R_tcp2base, T_tcp2base = rot_tran_from_tool_pose(tool_pose)
        H_tcp2base = make_homogeneous(R_tcp2base, T_tcp2base)
        
        
        marker_in_tcp = apply_transformation(H_marker_in_cam, H_cam2tcp)
        marker_in_base = apply_transformation(marker_in_tcp, H_tcp2base)
        
        # final realign
        origin = apply_transformation(camera.T_cam2tcp, H_tcp2base)
        desired_rotvec = d.align_camera_to_point(origin=origin, target=marker.position.reshape(3,), R_tcp2base=R_tcp2base, T_tcp2base=T_tcp2base.reshape(3,))
        final_pose = build_pose(T_tcp2base, desired_rotvec)
        
        robot.movej(TaskPose(final_pose), blocking=True)
        
        joint_positions, tool_pose = robot.read_joint_and_task_space_data()
        R_tcp2base, T_tcp2base = rot_tran_from_tool_pose(tool_pose)
        H_tcp2base = make_homogeneous(R_tcp2base, T_tcp2base)
        
        tvecs = []
        rvecs = []
        limit = 200
        # camera is now aligned. compute pose x times
        for i in range(limit):
            # look for aruco markers in the image
            detected_markers = d.detect_markers()
            
            # if no markers were found we continue to next pose
            if not detected_markers:
                continue
            
            # check if the targeted marker was one of the detected markers
            if not id in detected_markers:
                print("unable to find target dict in frame")
                break
        
            # estimate the pose wrt to the camera frame and store in list
            H_marker_in_cam, ok = d.estimate_marker_pose(detected_markers[id])
            if not ok:
                print("failed to compute pose for marker with ID:", id)
                continue

            rvec, tvec = extract_pose_and_orientation(H_marker_in_cam)
            # store for later use
            tvecs.append(tvec)
            rvecs.append(rvec)
            
        if len(tvecs) != limit:
            print(f"Unable to refine location of marker {id}")
            continue
        
        # est now contain x number of samples.
        # compute the mean (as new location) and return std. dev as means of accuracy indicator
        tvecs = np.array(tvecs, dtype=np.float64)
        mean_position = tvecs[:, :3].mean(axis=0)  
        rvecs = np.array(rvecs, dtype=np.float64)
        mean_orientation = rvecs[:, :3].mean(axis=0)  
        R_maker_in_cam, _ = cv.Rodrigues(mean_orientation)
        
        marker_in_cam_new = make_homogeneous(R_maker_in_cam, mean_position)
            
        # transform from camera to TCP
        marker_in_tcp = apply_transformation(marker_in_cam_new, H_cam2tcp)
        
        # transform from TCP to base            
        marker_in_base = apply_transformation(marker_in_tcp, H_tcp2base)
        
        
        print(f"Marker {id}'s new location: tvec", marker_in_base[0:3, 3])
        detected_ids[id]["H"] = marker_in_base
        detected_ids[id]["robot"] = tool_pose

# initial scan
detected_ids = scan()

# refine ids found during scan
refine(detected_ids)
    
H_plate_in_base = detected_ids[plate_id]["H"]
H_mantis_in_base = detected_ids[mantis_id]["H"]
print(H_plate_in_base)
print(H_mantis_in_base)
# compute correlation
H_mantis2plate = apply_transformation(H_plate_in_base, np.linalg.inv(H_mantis_in_base))

input("move mantis please :)")

# now that the setup has changed, we perform another scan
detected_ids = scan()

# and again, refine the positions
refine(detected_ids)

# now we extract the positions of mantis and compute the position of the plate
H_mantis_in_base_new = detected_ids[mantis_id]["H"]
H_plate_in_base_new = H_mantis_in_base_new @ H_mantis2plate
r, t = extract_pose_and_orientation(H_plate_in_base_new)

# finally, we can align TCP with the plate
final_pose = build_pose([t[0], t[1], t[2] + 0.15], [0, 3.1415, 0])
print("final pose", final_pose)
robot.movej(TaskPose(final_pose), blocking=True)
    
# align of TCP (long side of SBS plate)
joint_positions, tool_pose = robot.read_joint_and_task_space_data()
R_tcp2base, T_tcp2base = rot_tran_from_tool_pose(tool_pose)

R_plate2base, _ = cv.Rodrigues(r)
rvec1 = R_tcp2base @ np.array([1,0,0], dtype=np.float64)
rvec2 = R_plate2base @ np.array([1,0,0], dtype=np.float64)
fix_z = np.arccos(np.dot(rvec1, rvec2) / (np.linalg.norm(rvec1) * np.linalg.norm(rvec2)))
sgn = np.sign(np.dot(np.cross(rvec1, rvec2), np.array([0, 0, 1], dtype=np.float64)))
fix_z *= sgn
R_z, _ = cv.Rodrigues(np.array([0,0,fix_z], dtype=np.float64))
R_tcp2base = R_z @ R_tcp2base
tool_orientation, _ = cv.Rodrigues(R_tcp2base)
final_pose = build_pose(T_tcp2base, tool_orientation)
robot.movej(TaskPose(final_pose), blocking=True)

# make sure the gripper is open
robot.send_script(script=open_gripper, blocking=True)

# descent gripper
tcp_offset = -0.025
final_pose = build_pose([t[0], t[1], t[2] + tcp_offset], tool_orientation)
print("final pose", final_pose)
robot.movel(TaskPose(final_pose), a=0.25, blocking=True)
robot.send_script(script=close_gripper, blocking=True)

joint_positions, tool_pose = robot.read_joint_and_task_space_data()
final_pose = build_pose([t[0], t[1], t[2] + 0.15], tool_pose[3:6])
robot.movel(TaskPose(final_pose), a=0.25, blocking=True)


cv.destroyWindow("feed")
pipeline.stop()
