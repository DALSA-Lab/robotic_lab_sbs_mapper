"""
MapIt package initialization.
"""

from .detector import Detector
from .calib import CalibratedCamera, new_calibration
from .utils import *

__version__ = "0.1.0"
__author__ = "Jesper Thøgersen"
__email__ = "s203841@dtu.com"

__all__ = [
    "Detector",
    "CalibratedCamera",
    "new_calibration",
]
