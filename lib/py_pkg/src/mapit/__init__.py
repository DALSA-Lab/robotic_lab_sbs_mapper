"""MapIt package initialization."""

from .calib import CalibratedCamera
from .detector import Detector
from .models import Marker
from .utils import (
    apply_rotation_and_translation,
    make_homogeneous,
    apply_transformation,
    extract_pose_and_orientation,
    build_pose,
    rot_tran_from_tool_pose,
    look_at,
    axis_angle_align,
)

__version__ = "0.1.0"
__author__ = "Jesper Thøgersen"
__email__ = "s203841@dtu.dk"

__all__ = [
    "Detector",
    "CalibratedCamera",
    "Marker",
    "apply_rotation_and_translation",
    "make_homogeneous",
    "apply_transformation",
    "extract_pose_and_orientation",
    "build_pose",
    "rot_tran_from_tool_pose",
    "look_at",
    "axis_angle_align",
]
