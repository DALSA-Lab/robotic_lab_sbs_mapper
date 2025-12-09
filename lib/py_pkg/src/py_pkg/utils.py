import numpy as np

def normalise_tcp(p):
    #pointer_offset = 2 + 5 + 23 + 10
    pointer_offset = 31 # measured
    p = np.array(p, dtype=np.float64).copy()
    p[2] -= pointer_offset / 1000
    return p

def apply_rotation_and_translation(point: np.array, R_matrix: np.array, t_vector: np.array):
    #TODO assert to check that point is a 3D point
    #TODO assert to check that R_matrix is 3x3 matrix
    #TODO assert to check that t_vector is a 3D vector
    point = np.array(point, dtype=np.float64).copy()
    return R_matrix @ point + t_vector

def build_pose(tvec, rvec):
    pose = []
    pose[0:3] = tvec[0:3]
    pose[3:5] = rvec[0:3]
    return pose