import cv2 as cv
import numpy as np
from math import sqrt, pow, pi
from calib import CalibratedCamera

#TODO this should go somewhere else....
marker_length = 0.04
half = marker_length / 2

object_points = np.array([
    [-half, half, 0],
    [half, half, 0],
    [half, -half, 0],
    [-half, -half, 0]], dtype=np.float64)

class Detector:
   
    def __init__(self, ar_dict: int, params: cv.aruco.DetectorParameters, camera: CalibratedCamera):
        if ar_dict:
            self.dictionary = cv.aruco.getPredefinedDictionary(ar_dict)
        else:
            self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        
        if params:
            self.params = params
        else:
            self.params = cv.aruco.DetectorParameters()
            self.params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        
        self.detector = cv.aruco.ArucoDetector(dictionary=self.dictionary, detectorParams=self.params)
        
        self.camera = camera
        
        # compute a new optimal camera matrix to get a ROI 
        w, h = camera.size()
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(camera.intrinsics, camera.distortion_coefficients, (w,h), 0, (w,h))
        self.roi = roi
        
    def detect_markers(self):
        
        img = self.camera.grab_frame()
        
        corners, ids, _ = self.detector.detectMarkers(image=img)
        
        corners_dict = {}
        
        if ids is not None and len(ids) > 0:
            # correlate ids to corner 
            for index, id in enumerate(ids.flatten()):
                corners_dict[id] = corners[index]
                        
        return corners_dict
    
    def estimate_marker_pose(self, img_points):
        img_pts = img_points.reshape((4, 2)).astype(np.float64)
        
        rvec = np.array([], dtype=np.float64)
        tvec = np.array([], dtype=np.float64)

        ret, rvec, tvec = cv.solvePnP(object_points, img_pts, self.camera.intrinsics, self.camera.distortion_coefficients, rvec= rvec, tvec=tvec, flags=cv.SOLVEPNP_IPPE_SQUARE)
        if ret:
            tvec, tvec = cv.solvePnPRefineLM(object_points, img_pts, self.camera.intrinsics, self.camera.distortion_coefficients, rvec, tvec)
            tvec = tvec.reshape(3,)
            rvec = rvec.reshape(3,)
        return (tvec, rvec), ret
    
    def center_marker_in_frame(self, point, R_tcp2base, T_tcp2base):
        camera_offset = (self.camera.R_cam2tcp @ self.camera.T_cam2tcp)
        target = point - camera_offset # offset target point
        
        p0 = T_tcp2base
        pr = (R_tcp2base @ np.array([0,0,1], dtype=np.float64)) # vector in z-direction 
        v0 = pr/np.linalg.norm(pr)
        p1 = target.copy()
        v1 = (p1-p0)/np.linalg.norm(p1-p0)
        # Calculate the axis of rotation
        N = np.cross(v0, v1)
        
        # Handle parallel vectors case (norm is near zero)
        if np.linalg.norm(N) < 1e-6:
            if np.dot(v0, v1) > 0: # Vectors are parallel and pointing same way (0 rotation)
                R = np.eye(3)
            else: # Vectors are anti-parallel (180 degree rotation needed)
                # Choose an arbitrary perpendicular axis for 180 deg rotation
                N = np.cross(np.array([[1], [0], [0]], dtype=np.float64), v0)
                if np.linalg.norm(N) < 1e-6:
                    N = np.cross(np.array([[0], [1], [0]], dtype=np.float64), v0)
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

        # Apply rotation to the current orientation
        new_tool_orientation = R @ R_tcp2base
        # Convert rotation matrix to rotation vector (euler-angle) presentation
        R_tcp2base_new, _ = cv.Rodrigues(new_tool_orientation)
        return R_tcp2base_new.flatten()