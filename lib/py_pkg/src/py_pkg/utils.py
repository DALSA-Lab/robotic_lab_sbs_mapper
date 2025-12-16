"""utilities.py - A collection of helper functions."""
import numpy as np
from math import pi, sqrt

def apply_rotation_and_translation(point: np.array, R_matrix: np.array, t_vector: np.array):
    """Function to apply a rotation and translation to a 3D point.
    
    Parameters
    ----------
    point : numpy.array
        A 3D point of dimensions 3x1.
    R_matrix : numpy.array
        A 3x3 rotation matrix.
    t_vector : numpy.array
        A translation vector of dimensions 3x1
    
    Raises
    ------
    AssertionError
        If the shape of any input parameter is incorrect.
    """
    assert point.shape == t_vector.shape, "shape of point and t_vector should match"
    assert R_matrix.shape == (3,3), "R_matrix has incorrect shape, should be 3x3"
    
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
