from dataclasses import dataclass
import numpy as np
import cv2 as cv
import warnings
@dataclass
class CalibratedCamera:
    R_cam2tcp: np.array
    T_cam2tcp: np.array
    distortion_coefficients: np.array
    intrinsics: np.array
    frame_grabber: callable # device specific callable method for grapping a single frame from the camera device
    image_width: int
    image_height: int
    
    def size(self):
        """Get image size in pixels.

        Returns
        -------
        tuple
            Image size in pixels, as a touple of image width and height (w, h).

        """
        return (self.image_width, self.image_height)

    def grab_frame(self):
        """Wrapper to invoke the `.frame_grabber()` method.

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
    def new_from_config(cls, path: str, frame_grapper: callable):
        """Constructor to create instance from YAML config
        
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
        T_cam2tcp = fs.getNode("T_cam2tcp").mat().astype(np.float64).reshape(3,)
        fs.release()
        
        return cls(R_cam2tcp, T_cam2tcp, dist_coeffs, camera_matrix, frame_grapper, image_width, image_height)
    
    def export_to_config(self, path):
        """Export CalibratedCamera parameters to a YAML file.
        
        Parameters
        ----------
        path : string
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
    
    def set_frame_grabber(self, callback: callable):
        """Set the frame_grabber of a `CalibratedCamera` instance.
        
        Parameters
        ----------
        frame_grapper : callable
            A callable method to get an image frame.
        """
        self.frame_grabber = callback
        
        return

def new_calibration(images, R_gripper2base, t_gripper2base):
    """
    Perform lens and hand-eye calibration to obtain a CalibratedCamera object.
        
    Parameters
    ----------
    images : list of np.array
        List of input images of calibration board.        
    R_gripper2base : list of np.array
        List of rotation vectors of the robot pose at each image, following the image sequence order.
    t_gripper2base : list of np.array
        List of translation vectors of the robot pose at each image, following the image sequence order.
    
    Returns
    --------
    CalibratedCamera
        A `CalibratedCamera` object.
        
    Raises
    -----
    AssertionError
        If the length of the input lists does not match.
    RuntimeError
        If the camera calibration failed.
        
    Notes
    -----
    The returned object does not have a `frame_grabber` set. You must manually set this using the `.set_frame_grabber` function afterwards.
    """
    if len(images) != len(R_gripper2base) or len(images) != len(t_gripper2base):
        raise AssertionError("Length of input lists does not match")
    
    # these should be passable?
    chessboard_width = 5
    chessboard_height = 8
    chessboard_pattern = (chessboard_width, chessboard_height)
    square_size = 0.03 # unit m
    #
    
    # create "ground-truth" chessboard corners to compare against
    object = np.zeros((chessboard_width * chessboard_height, 3), np.float32) # OpenCV only support float32 for cameraCalibration
    object[:, :2] = np.mgrid[0:chessboard_width, 0:chessboard_height].T.reshape(-1, 2)*square_size
    
    image_points = []
    object_points = []
    
    reference_image = None
    
    for index, image in enumerate(images):
        gray =  cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        reference_image = gray
        
        ret, corners = cv.findChessboardCorners(image=gray, patternSize=chessboard_pattern)
        if ret:
            object_points.append(object)
            
            corners = cv.cornerSubPix(image=gray, corners=corners, winSize=(11,11), zeroZone=(-1, -1), criteria=(cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            image_points.append(corners.astype(np.float32)) # must also be float32 for cameraCalibration
        else:
            warnings.warn(f"Failed to find chessboard corners for image with index: {index}")
            
    calib_ok, camera_matrix, distance_coefficients, R_target2cam, t_target2cam = cv.calibrateCamera(
        objectPoints=object_points, 
        imagePoints=image_points,
        imageSize=reference_image.shape[::-1],
        cameraMatrix=None,
        distCoeffs=None
    )
    
    if not calib_ok:
        raise RuntimeError("Failed to find the camera intrinsics.")

    R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(
        R_gripper2base=R_gripper2base,
        t_gripper2base=t_gripper2base,
        R_target2cam=R_target2cam,
        t_target2cam=t_target2cam,
        method=cv.CALIB_HAND_EYE_TSAI)
    
    (h,w) = reference_image.shape
    return CalibratedCamera(
        distortion_coefficients=distance_coefficients,
        frame_grabber=None,
        image_height=h,
        image_width=w,
        intrinsics=camera_matrix,
        R_cam2tcp=R_cam2gripper,
        T_cam2tcp=t_cam2gripper.reshape(3,)
    )
