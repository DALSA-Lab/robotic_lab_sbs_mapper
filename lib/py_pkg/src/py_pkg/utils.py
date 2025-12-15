import numpy as np
from math import pi, sqrt

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
