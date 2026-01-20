"""
MapIt package initialization.
"""

from .detector import Detector
from .calib import CalibratedCamera
from .utils import *
from .models import Marker

__version__ = "0.1.0"
__author__ = "Jesper Thøgersen"
__email__ = "s203841@dtu.com"

__all__ = [
    "Detector",
    "CalibratedCamera",
    "new_calibration",
]
