from detector import Detector
import cv2 as cv
import numpy as np
from ur_commander import CustomURRobot, TaskPose, JointPositions
from utils import apply_rotation_and_translation, build_pose
from calib import CalibratedCamera, calibrate_hand_eye
import logging
import pyrealsense2 as rs
import glob
import re
import json
import time

R_cam2tcp = np.array([
    [0.7031712873773839, -0.7108982951179903, -0.01318160105478331],
    [0.7110113851610904, 0.7029476017317904, 0.01809639165225672],
    [-0.003598719124931573, -0.02209713143960507, 0.9997493515890874]
], dtype=np.float64)

T_cam2tcp = np.array( [0.01239759056065678,
 -0.0570573497099558,
 0.01802776294011691], dtype=np.float64
)

# Scanning poses for initial scan. Choose one
# # camera down poses (all)
# scanning_poses = [ { "joint_positions": [ 0.6977952122688293, -1.0602992934039612, -1.4912654161453247, -1.705547948876852, 1.210611343383789, -3.321655813847677 ], "tool_pose": [ 0.2993800031055965, 0.031895342573468335, 0.7117841121699572, 0.9818552213634536, 2.4047959286330856, 0.285425863512494 ] }, { "joint_positions": [ -0.8729541937457483, -1.0602955979159852, -1.4912405014038086, -1.7055322132506312, 1.2106153964996338, -3.3216283957110804 ], "tool_pose": [ 0.03190596191561175, -0.2993738551756937, 0.7117983633822628, 2.3970576954818497, 1.007168461924532, -0.2983277031191602 ] }, { "joint_positions": [ -2.443749729787008, -1.0602556031993409, -1.4911922216415405, -1.705536504785055, 1.210623025894165, -3.321659866963522 ], "tool_pose": [ -0.299349936155045, -0.03188706930817878, 0.7118296305163697, 2.6175750048248165, -1.0686848400603248, -0.769603213650489 ] } ]
# #camera down (middle)
# scanning_poses = [ { "joint_positions": [ -0.8729541937457483, -1.0602955979159852, -1.4912405014038086, -1.7055322132506312, 1.2106153964996338, -3.321655813847677], "tool_pose": [ 0.03190596191561175, -0.2993738551756937, 0.7117983633822628, 2.3970576954818497, 1.007168461924532, -0.2983277031191602 ] }     ]

# # camera up poses (all)
# scanning_poses = [{ "joint_positions": [ 0.9047755002975464, -0.7546993058970948, -1.6345456838607788, -1.948550363580221, 1.1272639036178589, 0.021549642086029053 ], "tool_pose": [ 0.20191074230302364, -0.026995101311212095, 0.6746109108610215, -2.621523280002422, 1.0623026120349728, -0.7706480355773961 ] }, { "joint_positions": [ -0.6660011450396937, -0.7546910804561158, -1.6345137357711792, -1.9486061535277308, 1.127304196357727, 0.02158975601196289 ], "tool_pose": [ -0.026999326841257804, -0.20190021020235677, 0.6746163179673335, 1.098714747660447, -2.5958627272645796, 0.7606304626288358 ] }, { "joint_positions": [ -2.2367990652667444, -0.7547184985927125, -1.634465217590332, -1.9485818348326625, 1.1272401809692383, 0.021537883207201958 ], "tool_pose": [ -0.20190690922512006, 0.027000489406809176, 0.6746391392247257, -0.9753587676216235, -2.407028598407151, 0.28358848354026267 ] }] # camera up
# # camera up (middle)
# scanning_poses = [{ "joint_positions": [ -0.6660011450396937, -0.7546910804561158, -1.6345137357711792, -1.9486061535277308, 1.127304196357727, 0.02158975601196289 ], "tool_pose": [ -0.026999326841257804, -0.20190021020235677, 0.6746163179673335, 1.098714747660447, -2.5958627272645796, 0.7606304626288358 ] }] # camera up

# # camera up v2 (all)
# scanning_poses = [{"joint_positions": [0.6918385028839111, -0.6575752657702942, -1.9687697887420654, -1.569751338367798, 1.1915313005447388, -0.19200021425356084], "tool_pose": [0.2146376824878809, -0.042200797116373444, 0.5843579687392579, -2.5614970394375955, 1.0616203461567262, -0.8268181953846278]}, {"joint_positions": [-0.878883186970846, -0.6575910013965149, -1.9687368869781494, -1.5697354596671467, 1.1915277242660522, -0.19193631807436162], "tool_pose": [-0.0421839279475117, -0.21464240435334403, 0.5843736342211288, 1.0671567747502646, -2.5778238850604787, 0.8451084227278082]}, {"joint_positions": [-2.4497361818896692, -0.6575675171664734, -1.968725323677063, -1.5696793359569092, 1.1914911270141602, -0.1919477621661585], "tool_pose": [-0.2146339859826663, 0.04220552190015343, 0.5843909934120658, -0.9745005320190984, -2.3510571718128235, 0.3313774139382412]}]
# # camera up v2 (middle)
scanning_poses = [{"joint_positions": [-0.878883186970846, -0.6575910013965149, -1.9687368869781494, -1.5697354596671467, 1.1915277242660522, -0.19193631807436162], "tool_pose": [-0.0421839279475117, -0.21464240435334403, 0.5843736342211288, 1.0671567747502646, -2.5778238850604787, 0.8451084227278082]}]


# own table
scanning_poses = [ { "joint_positions": [ 0.42052334547042847, -0.9788521093181153, -1.683270812034607, -2.03304447750234, 1.5599732398986816, -0.3855341116534632 ], "tool_pose": [ 0.24905170182472955, -0.034941429697866906, 0.5982163113793078, -2.909309877637324, 1.1661340537202702, -0.028255084240643095 ] }, { "joint_positions": [ -1.1501897017108362, -0.9789041441730042, -1.6833593845367432, -2.0330854854979457, 1.5599448680877686, -0.3855021635638636 ], "tool_pose": [ -0.03491267054139515, -0.24907942120074844, 0.5981612202657985, 1.2324610487710859, -2.8812154278516693, 0.027611297215269885 ] }, { "joint_positions": [ -2.720891062413351, -0.9788157504848023, -1.6833521127700806, -2.0330854854979457, 1.559952974319458, -0.38554555574526006 ], "tool_pose": [ -0.249046727387719, 0.03490422390117855, 0.5981833064030163, -1.1618668675587172, -2.8994618144408437, 0.011045965251221894 ] } ]
scanning_poses = [ { "joint_positions": [ 0.42052334547042847, -0.9788521093181153, -1.683270812034607, -2.03304447750234, 1.5599732398986816, -0.3855341116534632 ], "tool_pose": [ 0.24905170182472955, -0.034941429697866906, 0.5982163113793078, -2.909309877637324, 1.1661340537202702, -0.028255084240643095 ] }]

# 5 markers test
scanning_poses = [ { "joint_positions": [ -1.3773406187640589, -1.6655341587462367, -1.5182075500488281, -1.5289876957288762, 1.5669612884521484, -0.6065061728106897 ], "tool_pose": [ -0.02838596835398986, -0.5482883750379605, 0.469153264349949, 1.2231224504237515, -2.8936990677192846, 0.0001416807411630961 ] } ]

def extract_poses_from_file(fpath):
    if fpath == "":
        return None, False
    
    with open(fpath, 'r') as file:
        waypoints = json.load(file)

    R_gripper2Base = []
    t_gripper2Base = []
    for element in waypoints:
        if element["tool_pose"] != None:
            pose = element["tool_pose"]
            x, y, z = pose[0:3]
            rx, ry, rz = pose[3:6]

            rvec = np.array([rx,ry,rz], dtype=np.float64)

            R, _ = cv.Rodrigues(rvec)

            R_gripper2Base.append(R)
            t_gripper2Base.append(np.array([x,y,z], dtype=np.float64))
        
    return (R_gripper2Base, t_gripper2Base), True

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


# obtained camera intrinsics and distortion coeffs. should come from calib.py eventually #TODO    
camera_intrinsics = np.array([
    [ 1364.83618164062, 0.0, 971.963623046875],
    [0.0, 1364.55151367188, 575.387329101562],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

distortion_coefficients = np.array([
    0.0, 0.0, 0.0, 0.0, 0.0
], dtype=np.float64)


# Manually create
# camera = CalibratedCamera(
#     R_cam2tcp=R_cam2tcp,
#     T_cam2tcp=T_cam2tcp,
#     distortion_coefficients=distortion_coefficients,
#     intrinsics=camera_intrinsics,
#     frame_grapper=lambda: get_realsense_frame(pipeline, CV_NAMED_WINDOW),
#     image_height=1080,
#     image_width=1920,
# )


# Load from file
camera = CalibratedCamera.new_from_config("/home/jesper/DTU/KAND/calibrations/dec9/camera_calibration.yml", lambda: get_realsense_frame(pipeline, CV_NAMED_WINDOW))

# # Calibrate from images and poses
(R_gripper2Base, t_gripper2Base), ok = extract_poses_from_file("/home/jesper/DTU/KAND/ur_commander/examples/custom_waypoints.json")

# Load images from folder 
images_loc = glob.glob("/home/jesper/DTU/KAND/ur_commander/*.jpg")
images_loc = sorted(images_loc, key=lambda x: int(re.findall(r'\d+', x)[-1]))
images = []
for fname in images_loc:
    image = cv.imread(fname)
    images.append(image)
    
camera = calibrate_hand_eye(images, R_gripper2Base, t_gripper2Base)
camera.set_frame_grabber(lambda: get_realsense_frame(pipeline, CV_NAMED_WINDOW))

robot = CustomURRobot("192.168.1.102", logging.INFO)

d = Detector(ar_dict=cv.aruco.DICT_4X4_50, params=None, camera=camera)


# initial scan
global_ids = {}
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
    R_tcp2base, _ = cv.Rodrigues(np.array(tool_pose[3:6], dtype=np.float64))
    T_tcp2base = np.array(tool_pose[0:3], dtype=np.float64).copy()
    
    # for each id, compute an initial pose
    for id, corners in detected_markers.items():
        print("target ID:", id) 
        
        img_points = corners
        (t_vector, r_vector), ok = d.estimate_marker_pose(img_points)
        if not ok:
            print("failed to compute pose for marker with ID: ", )
        
        print(f"target ID: {id}'s distance from camera:\t{t_vector} with r_vector: {r_vector}")
        
        # transform from camera to TCP
        marker_in_tcp = (apply_rotation_and_translation(t_vector, R_cam2tcp, T_cam2tcp))
        print(f"target ID: {id} located at (TCP space):\t{marker_in_tcp}")
        
        # transform from TCP to base            
        marker_in_base = apply_rotation_and_translation(marker_in_tcp, R_tcp2base, T_tcp2base)
        print(f"target ID: {id} located at (BASE space):\t{marker_in_base}")
        
        # store for later use
        obj = {"pose": marker_in_base, "ori": r_vector, "robot": tool_pose}
        global_ids[id] = obj


# refinement of pose for each marker
for id in global_ids:
    obj = global_ids[id]
    tvec = obj["pose"]
    rvec = obj["ori"]
    
    tool_pose = obj["robot"]
    
    R_tcp2base, _ = cv.Rodrigues(np.array(tool_pose[3:6], dtype=np.float64))
    T_tcp2base = np.array(tool_pose[0:3], dtype=np.float64).copy()
        
    # compute the required tool rotation to align camera to marker
    desired_rotvec = d.center_marker_in_frame(point=tvec.reshape(3,), R_tcp2base=R_tcp2base, T_tcp2base=T_tcp2base.reshape(3,))
    
    # align camera to marker center at scanning pose
    tool_pose = build_pose(T_tcp2base, desired_rotvec)    
    #robot.movej(TaskPose(pose), blocking=True)
    
    #joint_positions, tool_pose = robot.read_joint_and_task_space_data()
    R_tcp2base, _ = cv.Rodrigues(np.array(tool_pose[3:6], dtype=np.float64))
    T_tcp2base = np.array(tool_pose[0:3], dtype=np.float64).copy()

    depth = np.sqrt(np.power(tvec[0] - T_tcp2base[0],2)+np.power(tvec[1] - T_tcp2base[1],2)+np.power(tvec[2] - T_tcp2base[2],2))
    z = 0
    if depth > 0.25:
        z = depth - 0.3
    v = np.array([[0],[0],[1]])
    v = (v / np.linalg.norm(v) ) * z
    v_in_base = T_tcp2base + (R_tcp2base @ v.flatten())
    
    new_pose = build_pose(v_in_base, desired_rotvec)    
    robot.movej(TaskPose(new_pose), blocking=True)
    
    joint_positions, tool_pose = robot.read_joint_and_task_space_data()
    R_tcp2base, _ = cv.Rodrigues(np.array(tool_pose[3:6], dtype=np.float64))
    T_tcp2base = np.array(tool_pose[0:3], dtype=np.float64).copy()
    
    est = []
    limit = 200
    # camera is now aligned. compute pose x times
    for i in range(limit):
        # look for aruco markers in the image
        detected_markers = d.detect_markers()
        
        # if no corners were found we did not find any markers so we continue to next pose
        if not detected_markers:
            continue
        
        if not id in detected_markers:
            print("unable to find target dict in frame")
            break
    
        (t_vector, r_vector), ok = d.estimate_marker_pose(detected_markers[id])
        if not ok:
            print("failed to compute pose for marker with ID: ", )
                    
        # store for later use
        est.append(t_vector)
        
    if len(est) != limit:
        print(f"Unable to refine location of marker {id}")
        continue
    
    # est now contain x number of samples.
    # compute the mean (as new location) and return std. dev as means of accuracy indicator
    est = np.array(est, dtype=np.float64)
    mean_position = est[:, :3].mean(axis=0)    
    print(f"Marker {id}'s mean distance from camera:", mean_position)
    
    # transform from camera to TCP
    marker_in_tcp = (apply_rotation_and_translation(mean_position, R_cam2tcp, T_cam2tcp))
    
    #marker_in_tcp[2] += 0.005
    
    # transform from TCP to base            
    marker_in_base = apply_rotation_and_translation(marker_in_tcp, R_tcp2base, T_tcp2base)
    print(f"Marker {id}'s new location:", marker_in_base)
        
        

        
cv.destroyWindow("feed")
pipeline.stop()
