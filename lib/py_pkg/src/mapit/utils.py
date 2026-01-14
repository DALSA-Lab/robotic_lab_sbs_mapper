"""utilities.py - A collection of helper functions."""
import numpy as np
from math import pi, sqrt
import cv2 as cv
from ur_commander import TaskPose

ALIGN_AXIS_ANGLE = 0
ALIGN_LOOK_AT = 1

def apply_rotation_and_translation(point: np.array, R_matrix: np.array, t_vector: np.array) -> np.array:
    """Function to apply a rotation and translation to a 3D point.
    
    Parameters
    ----------
    point : numpy.array
        A 3D point of dimensions 3x1.
    R_matrix : numpy.array
        A 3x3 rotation matrix.
    t_vector : numpy.array
        A translation vector of dimensions 3x1
        
    Returns
    -------
    numpy.narray
        The transformed 3D point.
    
    Raises
    ------
    AssertionError
        If the shape of any input parameter is incorrect.
    """
    assert point.shape == t_vector.shape, "shape of point and t_vector should match"
    assert R_matrix.shape == (3,3), "R_matrix has incorrect shape, should be 3x3"
    
    point = np.array(point, dtype=np.float64).copy()
    return R_matrix @ point + t_vector

def make_homogeneous(R_matrix: np.array, t_vector: np.array) -> np.array:
    assert t_vector.shape == (3,), "t_vector has incorrect shape, should be 3x1"
    
    if R_matrix.shape == (3,):
        R_matrix, _ = cv.Rodrigues(R_matrix)
    
    assert R_matrix.shape == (3,3), "R_matrix has incorrect shape, should be 3x3"
    
    H =np.block([
        [R_matrix, t_vector.reshape(-1, 1)], # Add t as a column vector
        [0, 0, 0, 1] 
    ])
    return H

def apply_transformation(target: np.array, transformation: np.array) -> np.array:
    """Function to apply a 4x4 homogeneous transformation to another 4x4 homogeneous transformation.
    
    Parameters
    ----------
    target : numpy.array
        A 4x4 homogeneous transformation matrix.
    transformation : numpy.array
        A 4x4 homogeneous transformation matrix.
        
    Returns
    -------
    numpy.narray
        The transformed target.
    
    Raises
    ------
    AssertionError
        If the shape of any input parameter is incorrect.
    """
    
    assert transformation.shape == (4,4), "transformation has incorrect shape, should be 4x4"
    assert (target.shape == (4,4) or target.shape == (3,)), f"target has incorrect shape{target.shape}, should be either 4x4 or 3x1"
    
    if target.shape == (3,):
        target = np.append(target,[1])
        
        new_target = transformation @ target

        return new_target[0:3]
   
    return transformation @ target

def extract_pose_and_orientation(H_matrix: np.array) -> tuple[np.ndarray, np.ndarray]:
    assert H_matrix.shape == (4,4), "H_matrix has incorrect shape, should be 4x4"
    
    R_matrix = H_matrix[0:3, 0:3]
    R_vector, _ = cv.Rodrigues(R_matrix)
    
    t_vector = H_matrix[0:3, 3]
    
    return (R_vector.flatten(), t_vector)

def build_pose(tvec, rvec):
    pose = []
    pose[0:3] = tvec[0:3]
    pose[3:5] = rvec[0:3]
    return pose

def rot_tran_from_tool_pose(tool_pose: TaskPose):
    assert len(tool_pose) == 6, "Invalid tool_pose. Unable to extract rotation and translation."
    R_tcp2base, _ = cv.Rodrigues(np.array(tool_pose[3:6], dtype=np.float64))
    T_tcp2base = np.array(tool_pose[0:3], dtype=np.float64)
    return (R_tcp2base, T_tcp2base)

def compute_approach(self, point, orientation, distance):
    # create a normalized vector from point to robot origo
    v = -point/np.linalg.norm(point)
    
    # compute the z coordiante (b side of triangle)
    z = (distance/np.sin(pi/2))*np.sin(pi/3)
    
    # compute the scaling factor for the normalised vector
    s = z / sqrt(pow(v[0], 2) + pow(v[1], 2))
    
    # create the final target point
    target = point + (v*s)
    
    #replace the z coordinate
    target[2] = z
    
    
    return target

def look_at(camera_position, target_position):
        forward = target_position - camera_position
        forward = forward / np.linalg.norm(forward)
        
        world_up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        if (abs(np.dot(world_up, forward)) > 0.99):
                world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        
        right = np.cross(forward, world_up)
        right = right/np.linalg.norm(right)
        
        up = np.cross(forward, right)
        up = up/np.linalg.norm(up)
        
        R = np.array([right, up, forward], dtype=np.float64).T
        
        return R
        
def axis_angle_align(camera_position, target, R_cam2base):
    v0 = (R_cam2base @ np.array([0,0,1], dtype=np.float64)) # vector in z-direction  
    v0 = v0/np.linalg.norm(v0)
    v1 = target - camera_position
    v1 = v1/np.linalg.norm(v1)
    # Calculate the axis of rotation
    N = np.cross(v0, v1)
    
    # Handle parallel vectors case (norm is near zero)
    if np.linalg.norm(N) < 1e-6:
        if np.dot(v0, v1) > 0: # Vectors are parallel and pointing same way (0 rotation)
            R = np.eye(3)
        else: # Vectors are anti-parallel (180 degree rotation needed)
            # Choose an arbitrary perpendicular axis for 180 deg rotation
            N = np.cross(np.array([1.0, 0.0, 0.0], dtype=np.float64), v0)
            if np.linalg.norm(N) < 1e-6:
                N = np.cross(np.array([0.0, 1.0, 0.0], dtype=np.float64), v0)
            N = N / np.linalg.norm(N)
            theta = pi # 180 degrees
    else:
        N = N/np.linalg.norm(N) # Normalize N
        # Clamp the dot product to [-1, 1] range to avoid complex numbers due to
        # floating point inaccuracies (e.g., dot product slightly > 1 or < -1)
        dot_prod = np.dot(v0, v1)
        dot_prod = max(-1.0, min(1.0, dot_prod))
        theta = np.arccos(dot_prod)
    ### Rodrigues' rotation formula implementation
    # skew-symmetric matrix from the N vector
    N_skew = np.array([
            [  0,    -N[2],  N[1]],
            [N[2],   0,     -N[0]],
            [-N[1],  N[0],   0]
        ], dtype=np.float64)

    N_col = N.reshape((3,1))
    # the outer product matrix (N*transpose(N))
    N_outer = N_col @ np.transpose(N_col)

    # The complete rotation matrix R
    R = np.cos(theta)*np.eye(3) + np.sin(theta)*N_skew + (1-np.cos(theta))*N_outer
    
    return R