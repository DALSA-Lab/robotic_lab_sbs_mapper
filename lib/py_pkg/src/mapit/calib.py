"""cailb.py - Camera calibration module."""

import warnings
from dataclasses import dataclass
from math import atan2, pi, sqrt
from typing import Callable

import cv2 as cv
import numpy as np


@dataclass
class CalibratedCamera:
    """Collects camera-related parameters in a single object.

    Parameters
    ----------
    R_cam2tcp : numpy.ndarray
        Rotation matrix from camera to TCP.
    T_cam2tcp : numpy.ndarray
        Translation vector from camera to TCP.
    distortion_coefficients : numpy.ndarray
        Camera distortion coefficients.
    intrinsics : numpy.ndarray
        Camera intrinsic matrix.
    frame_grabber : callable
        Device-specific callable that grabs a single frame.
    image_width : int
        Image width in pixels.
    image_height : int
        Image height in pixels.

    """

    R_cam2tcp: np.ndarray
    T_cam2tcp: np.ndarray
    distortion_coefficients: np.ndarray
    intrinsics: np.ndarray
    frame_grabber: Callable[[], np.ndarray]
    image_width: int
    image_height: int

    def size(self):
        """Get image size in pixels.

        Returns
        -------
        tuple
            Image size in pixels as (width, height).

        """
        return (self.image_width, self.image_height)

    def grab_frame(self):
        """Invoke the `.frame_grabber()` method.

        Raises
        ------
        AssertionError
            If `frame_grabber` is not callable, i.e. not set or configured incorrectly.

        """
        if callable(self.frame_grabber):
            return self.frame_grabber()
        else:
            raise AssertionError("frame_grabber not callable.")

    @classmethod
    def new_calibration(
        cls,
        images,
        R_gripper2base,
        t_gripper2base,
        board: tuple,
        frame_grapper: Callable[[], np.ndarray],
    ):
        """Perform lens and hand-eye calibration to instantiate a CalibratedCamera instance.

        Parameters
        ----------
        images : list of numpy.array
            List of input images of calibration board.
        R_gripper2base : list of numpy.array
            List of rotation vectors of the robot pose at each image, following the image sequence order.
        t_gripper2base : list of numpy.array
            List of translation vectors of the robot pose at each image, following the image sequence order.
        board : tuple
            A tuple containing the number of inner chessboard-corners  (width, height) and square size (mm), e.g (5, 8, 0.03).
        frame_grapper : callable
            A callable method to get an image frame.

        Returns
        -------
        CalibratedCamera
            A `CalibratedCamera` object.

        Raises
        ------
        AssertionError
            If the length of the input lists does not match.
        RuntimeError
            If the camera calibration failed.

        """
        if len(images) != len(R_gripper2base) or len(images) != len(t_gripper2base):
            raise AssertionError("Length of input lists does not match")

        assert len(board) == 3, "Incorrect number of chessboard identifiers"
        chessboard_width, chessboard_height, square_size = board

        chessboard_pattern = (chessboard_width, chessboard_height)

        # create "ground-truth" chessboard corners to compare against
        object = np.zeros(
            (chessboard_width * chessboard_height, 3), np.float32
        )  # OpenCV only support float32 for cameraCalibration
        object[:, :2] = (
            np.mgrid[0:chessboard_width, 0:chessboard_height].T.reshape(-1, 2)
            * square_size
        )

        image_points = []
        object_points = []

        reference_image = np.ndarray([])

        for index, image in enumerate(images):
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            reference_image = gray

            ret, corners = cv.findChessboardCorners(
                image=gray, patternSize=chessboard_pattern
            )
            if ret:
                object_points.append(object)

                corners = cv.cornerSubPix(
                    image=gray,
                    corners=corners,
                    winSize=(11, 11),
                    zeroZone=(-1, -1),
                    criteria=(
                        cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,
                        30,
                        0.001,
                    ),
                )
                image_points.append(
                    corners.astype(np.float32)
                )  # must also be float32 for cameraCalibration
            else:
                warnings.warn(
                    f"Failed to find chessboard corners for image with index: {index}",
                    stacklevel=2
                )

        calib_ok, camera_matrix, distortion_coefficients, R_target2cam, t_target2cam = (
            cv.calibrateCamera(
                objectPoints=object_points,
                imagePoints=image_points,
                imageSize=reference_image.shape[::-1],
                cameraMatrix=None,
                distCoeffs=None,
            )
        )

        if not calib_ok:
            raise RuntimeError("Failed to find the camera intrinsics.")

        R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            method=cv.CALIB_HAND_EYE_PARK,
        )

        # compute reprojection error
        total_error = 0.0
        total_points = 0

        for i in range(len(object_points)):
            imgpoints2, _ = cv.projectPoints(
                object_points[i],
                R_target2cam[i],
                t_target2cam[i],
                camera_matrix,
                distortion_coefficients,
            )

            error = cv.norm(image_points[i], imgpoints2, cv.NORM_L2)
            total_error += error**2
            total_points += len(object_points[i])

        reproj_error = np.sqrt(total_error / total_points)

        print(f"[Camera Calibration] RMS reprojection error: {reproj_error:.3f} px")

        roll = atan2(R_cam2gripper[2][1], R_cam2gripper[2][2])
        pitch = atan2(
            -R_cam2gripper[2][0],
            sqrt(pow(R_cam2gripper[0][0], 2) + pow(R_cam2gripper[1][0], 2)),
        )
        yaw = atan2(R_cam2gripper[1][0], R_cam2gripper[0][0])
        print("roll (rotation around x axis)  rad:", roll, "deg:", (180.0 * roll) / pi)
        print(
            "pitch (rotation around y axis)  rad:", pitch, "deg:", (180.0 * pitch) / pi
        )
        print("yaw (rotation around z axis)  rad:", yaw, "deg:", (180.0 * yaw) / pi)

        (h, w) = reference_image.shape
        return cls(
            R_cam2gripper,
            t_cam2gripper,
            distortion_coefficients,
            camera_matrix,
            frame_grapper,
            w,
            h,
        )

    @classmethod
    def new_from_config(cls, path: str, frame_grapper: Callable[[], np.ndarray]):
        """Create a `CalibratedCamera` from a YAML configuration file.

        Parameters
        ----------
        path : str
            The path to the YAML config file.
        frame_grapper : callable
            A callable method to get an image frame.


        Returns
        -------
        CalibratedCamera
            A `CalibratedCamera` object.

        """
        # open YAML file using OpenCV's FileStore since it supports converting OpenCV matrix to numpy
        fs = cv.FileStorage(path, cv.FILE_STORAGE_READ)

        camera_matrix = fs.getNode("camera_matrix").mat().astype(np.float64)
        dist_coeffs = fs.getNode("distortion_coefficients").mat().astype(np.float64)
        image_width = int(fs.getNode("image_width").real())
        image_height = int(fs.getNode("image_height").real())
        R_cam2tcp = fs.getNode("R_cam2tcp").mat().astype(np.float64)
        T_cam2tcp = (
            fs.getNode("T_cam2tcp")
            .mat()
            .astype(np.float64)
            .reshape(
                3,
            )
        )
        fs.release()

        return cls(
            R_cam2tcp,
            T_cam2tcp,
            dist_coeffs,
            camera_matrix,
            frame_grapper,
            image_width,
            image_height,
        )

    def export_to_config(self, path):
        """Export CalibratedCamera parameters to a YAML file.

        Parameters
        ----------
        path : str
            The path to the YAML config file destination.

        """
        fs = cv.FileStorage(path, cv.FileStorage_WRITE)
        fs.write("camera_matrix", self.intrinsics)
        fs.write("distortion_coefficients", self.distortion_coefficients)
        fs.write("image_width", self.image_width)
        fs.write("image_height", self.image_height)
        fs.write("R_cam2tcp", self.R_cam2tcp)
        fs.write("T_cam2tcp", self.T_cam2tcp)
        fs.release()

        return

    def set_frame_grabber(self, frame_grabber: Callable[[], np.ndarray]):
        """Set the frame_grabber of a `CalibratedCamera` instance.

        Parameters
        ----------
        frame_grapper : callable
            A callable method to get an image frame.

        """
        self.frame_grabber = frame_grabber

        return
