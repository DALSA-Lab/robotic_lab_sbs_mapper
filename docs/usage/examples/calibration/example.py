# imports
import glob
import json
import re

import cv2
import numpy as np

from mapit import CalibratedCamera


# list of robot poses
def extract_poses_from_file(fpath):
    R_gripper2base = []
    t_gripper2base = []
    if fpath == "":
        return (R_gripper2base, t_gripper2base), False

    with open(fpath, "r") as file:
        poses = json.load(file)

    for pose in poses:
        if pose["tool_pose"] != None:
            tool_pose = pose["tool_pose"]
            x, y, z = tool_pose[0:3]
            rx, ry, rz = tool_pose[3:6]

            rvec = np.array([rx, ry, rz], dtype=np.float64)

            R, _ = cv2.Rodrigues(rvec)

            R_gripper2base.append(R)
            t_gripper2base.append(np.array([x, y, z], dtype=np.float64))

    return (R_gripper2base, t_gripper2base), True


# arrays to store rotation matrixes and translation vectors
source_folder = "/home/jesper/DTU/KAND/calibrations/gripper-dec22-lights-on-close/"
(R_gripper2base, t_gripper2base), ok = extract_poses_from_file(
    source_folder + "custom_waypoints_gripper_new_close.json"
)

# load images from folder
# select all files matching the .jpg extension
images_loc = glob.glob(source_folder + "*.jpg")
# sort images based on their filename
images_loc = sorted(images_loc, key=lambda x: int(re.findall(r"\d+", x)[-1]))
images = []
for fname in images_loc:
    image = cv2.imread(fname)
    images.append(image)

board_height = 5
board_width = 8
square_size_m = 0.03  # 30mm square side length
board = (board_height, board_width, square_size_m)

# prepare camera capture callable
cam = cv2.VideoCapture(0)
def grab_frame(cam):
    _, frame = cam.read()
    img = np.asanyarray(frame.grab_data())
    return img

# create new CalibratedCamera object
camera = CalibratedCamera.new_calibration(
    images, R_gripper2base, t_gripper2base, board, grab_frame(cam)
)
print(camera)
