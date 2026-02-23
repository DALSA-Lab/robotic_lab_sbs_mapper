from typing import Optional

import cv2 as cv
import numpy as np

from .calib import CalibratedCamera
from .utils import *

# TODO this should go somewhere else....
marker_length = 0.036
half = marker_length / 2

object_points = np.array(
    [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
    dtype=np.float64,
)


class Detector:
    """ArUco marker detector wrapper with camera-aware pose estimation support.

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

    def __init__(
        self,
        aruco_dict: int,
        aruco_params: Optional[cv.aruco.DetectorParameters],
        camera: CalibratedCamera,
    ):
        if aruco_dict:
            self.dictionary = cv.aruco.getPredefinedDictionary(aruco_dict)
        else:
            self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)

        if aruco_params:
            self.aruco_params = aruco_params
        else:
            self.aruco_params = cv.aruco.DetectorParameters()
            self.aruco_params.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX

        self.detector = cv.aruco.ArucoDetector(
            dictionary=self.dictionary, detectorParams=self.aruco_params
        )

        self.camera = camera

        # compute a new optimal camera matrix to get a ROI
        w, h = camera.size()
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(
            camera.intrinsics, camera.distortion_coefficients, (w, h), 0, (w, h)
        )
        self.roi = roi

    def detect_markers(self) -> dict:
        """Capture an image frame and detect ArUco markers.

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
        """Estimate marker pose in world coordinate frame, with respect to the camera frame.

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
            A np.array containing the translation vector and rotation (wrt. camera origin),
            represented as a homogeneous transformation matrix.
            A bool reflecting the state of the `solvePnP` (false if `solvePnP` failed)

        """
        img_pts = img_points.reshape((4, 2)).astype(np.float64)

        rvec = np.array([], dtype=np.float64)
        tvec = np.array([], dtype=np.float64)

        H_marker_in_cam = np.eye(4)
        ret, rvec, tvec = cv.solvePnP(
            object_points,
            img_pts,
            self.camera.intrinsics,
            self.camera.distortion_coefficients,
            rvec=rvec,
            tvec=tvec,
            flags=cv.SOLVEPNP_IPPE_SQUARE,
        )
        if ret:
            tvec, tvec = cv.solvePnPRefineLM(
                object_points,
                img_pts,
                self.camera.intrinsics,
                self.camera.distortion_coefficients,
                rvec,
                tvec,
            )
            tvec = tvec.reshape(
                3,
            )
            rvec = rvec.reshape(
                3,
            )
            H_marker_in_cam = make_homogeneous(rvec, tvec)
        return H_marker_in_cam, ret

    def align_camera_to_point(
        self, origin, target, R_tcp2base, T_tcp2base, method: int = ALIGN_AXIS_ANGLE
    ):
        """Function to generate a new robot orientation to align the camera optical center with a 3D point.

        Parameters
        ----------
        point : numpy.array
            A 3D point of dimensions 3x1, describing the camera position in the world frame.
        point : numpy.array
            A 3D point of dimensions 3x1, describing the target in the world frame.
        R_tcp2base : numpy.array
            A 3x3 rotation matrix to transfrom from robot tool to robot base coordinate.
        T_tcp2base : numpy.array
            A 3x1 translation vector to transform from robot tool to robot base coordinate.
        Method : int
            Flag to select which method to use for aligning camera. Defaults to ALIGN_AXIS_ANGLE.

        Returns
        -------
        numpy.narray
            A rotation vector, which when applied to the robot tool aligns the camera to the desired point.

        """
        if not (method == ALIGN_AXIS_ANGLE or method == ALIGN_LOOK_AT):
            raise AssertionError(f"Invalid align method requested: {method}")

        # origin = apply_rotation_and_translation(self.camera.T_cam2tcp, R_tcp2base, T_tcp2base)
        if method == ALIGN_AXIS_ANGLE:
            R_cam2base = R_tcp2base @ self.camera.R_cam2tcp
            R = axis_angle_align(origin, target, R_cam2base)
        elif method == ALIGN_LOOK_AT:
            R = look_at(origin, target)

        # R now contains a rotation matrix.
        # If R was computed using axis-angle, we need to transform from tcp frame to world:
        if method == ALIGN_AXIS_ANGLE:
            R = R @ R_tcp2base

        # Convert rotation matrix to rotation vector (euler-angle) presentation
        R_tcp2base_new, _ = cv.Rodrigues(R)
        R_tcp2base_new = R_tcp2base_new.flatten()
        return R_tcp2base_new
