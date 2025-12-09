from dataclasses import dataclass
import numpy as np

@dataclass
class CalibratedCamera:
    R_cam2tcp: np.array
    T_cam2tcp: np.array
    distortion_coefficients: np.array
    intrinsics: np.array
    frame_grapper: callable # device specific callable method for grapping a single frame from the camera device
    
    def size(self):
        return (1920, 1080)

    def grab_frame(self):
        return self.frame_grapper()