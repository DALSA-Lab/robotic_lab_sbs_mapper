import cv2 as cv
import numpy as np
from math import sqrt, pow, pi
from .calib import CalibratedCamera

#TODO this should go somewhere else....
marker_length = 0.04
half = marker_length / 2

object_points = np.array([
    [-half, half, 0],
    [half, half, 0],
    [half, -half, 0],
    [-half, -half, 0]], dtype=np.float64)

class Detector:
    """
    ArUco marker detector wrapper with camera-aware pose estimation support.

    This class employ OpenCV's `cv.aruco.ArucoDetector` and binds it
    together with camera calibration data to provide a convenient interface for ArUco marker detection.

    Parameters
    ----------
    ar_dict : int
        OpenCV ArUco dictionary identifier (e.g. `cv.aruco.DICT_4X4_50`).
        If `None` or `0`, `DICT_4X4_50` is used as default.
    aruco_params : cv.aruco.DetectorParameters
        Detector parameters controlling thresholding, refinement, and
        detection behavior. If `None`, default parameters are created
        with sub-pixel corner refinement enabled.
    camera : CalibratedCamera
        A `CalibratedCamera` instance.

    Attributes
    ----------
    dictionary : cv.aruco.Dictionary
        The ArUco dictionary used for marker detection.
    aruco_params : cv.aruco.DetectorParameters
        Detector configuration parameters.
    detector : cv.aruco.ArucoDetector
        OpenCV ArUco detector instance.
    camera : CalibratedCamera
        CalibratedCamera object used for pose estimation.
    roi : tuple[int, int, int, int]
        Region of interest returned by ``cv.getOptimalNewCameraMatrix``
        for cropping image frames.
    """
    def __init__(self, aruco_dict: int, aruco_params: cv.aruco.DetectorParameters, camera: CalibratedCamera):
        if aruco_dict:
            self.dictionary = cv.aruco.getPredefinedDictionary(aruco_dict)
        else:
            self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        
        if aruco_params:
            self.aruco_params = aruco_params
        else:
            self.aruco_params = cv.aruco.DetectorParameters()
            self.aruco_params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        
        self.detector = cv.aruco.ArucoDetector(dictionary=self.dictionary, detectorParams=self.aruco_params)
        
        self.camera = camera
        
        # compute a new optimal camera matrix to get a ROI 
        w, h = camera.size()
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(camera.intrinsics, camera.distortion_coefficients, (w,h), 0, (w,h))
        self.roi = roi
        
    def detect_markers(self) -> dict:
        """
        Capture an image frame and detect ArUco markers.

        This method takes a single image from the associated camera and
        detects ArUco markers using OpenCV's ``ArucoDetector``.

        Returns
        -------
        dict[int, np.ndarray]
            Dictionary mapping marker IDs to their detected corner coordinates.
            Each value is a `(4, 2)` array of image pixel coordinates in the
            order returned by OpenCV (clockwise starting from top-left).

            If no markers are detected, an empty dictionary is returned.

        Notes
        -----
        - Corner coordinates are given in **image pixel coordinates**, not in the camera coordinate frame.
        - No pose estimation or distortion correction is performed here. See `estimate_marker_pose()`.
        
        """

        img = self.camera.grab_frame()
        
        corners, ids, _ = self.detector.detectMarkers(image=img)
        
        corners_dict = {}
        
        # correlate ids to corner 
        if ids is not None and len(ids) > 0:
            for index, id in enumerate(ids.flatten()):
                corners_dict[id] = corners[index]
                        
        return corners_dict
    
    def estimate_marker_pose(self, img_points) -> tuple: 
        """
        Estimate marker pose in world coordinate frame, with respect to the camera frame.

        This method a single (4,2) array of ArUco marker corners, and estimates the
        position and orientation using Perspective-n-Point (PnP).
        
        Parameters
        ----------
        img_points : numpy.ndarray
            A single (4, 2) array-like structure representing the pixel 
            coordinates of the 4 marker corners in the image frame. This can be obtained from `detect_markers()`        

        Returns
        -------
        tuple
            A tuple containing the translation vector (wrt. camera origin) and rotation vector 
            (Rodrigues notation) of the marker's coordinate orientation.
            A bool reflecting the state of the `solvePnP` (false if `solvePnP` failed)
        """
        img_pts = img_points.reshape((4, 2)).astype(np.float64)
        
        rvec = np.array([], dtype=np.float64)
        tvec = np.array([], dtype=np.float64)

        ret, rvec, tvec = cv.solvePnP(object_points, img_pts, self.camera.intrinsics, self.camera.distortion_coefficients, rvec= rvec, tvec=tvec, flags=cv.SOLVEPNP_IPPE_SQUARE)
        if ret:
            tvec, tvec = cv.solvePnPRefineLM(object_points, img_pts, self.camera.intrinsics, self.camera.distortion_coefficients, rvec, tvec)
            tvec = tvec.reshape(3,)
            rvec = rvec.reshape(3,)
        return (tvec, rvec), ret
    
    def align_camera_to_point(self, point, R_tcp2base, T_tcp2base):
        """
        Function to generate a new robot orientation to align the camera center with a 3D point.
        
        Parameters
        ----------
        point : numpy.array
            A 3D point of dimensions 3x1.
        R_tcp2base : numpy.array
            A 3x3 rotation matrix to transfrom from robot tool to robot base coordinate.
        T_tcp2base : numpy.array
            A 3x1 translation vector to transform from robot tool to robot base coordinate.
        
        Returns
        -------
        numpy.narray
            A rotation vector, which when applied to the robot tool aligns the camera to the desired point.
         
        """
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