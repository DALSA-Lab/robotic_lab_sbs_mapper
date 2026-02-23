"""MapIt package initialization."""

from .calib import CalibratedCamera
from .detector import Detector
from .models import Marker
from .utils import *

__version__ = "0.1.0"
__author__ = "Jesper Thøgersen"
__email__ = "s203841@dtu.dk"

__all__ = [
    "Detector",
    "CalibratedCamera",
    "Marker",
]
