"""Data model definitions used throughout the toolbox.

This module currently defines the ``Marker`` class, which represents
a 3D marker described by a homogeneous transformation matrix.
"""

from dataclasses import dataclass

import numpy as np

ALIGN_AXIS_ANGLE = 0
ALIGN_LOOK_AT = 1


@dataclass
class Marker:
    """Represents a 3D marker with pose information.

    A marker is defined by a unique identifier and a 4x4 homogeneous
    transformation matrix encoding its position and orientation in space.

    Parameters
    ----------
    id : int
        Unique identifier of the marker.
    H : numpy.ndarray
        A (4, 4) homogeneous transformation matrix representing the
        marker pose.

    Attributes
    ----------
    id : int
        Unique identifier of the marker.
    H : numpy.ndarray
        Homogeneous transformation matrix of shape (4, 4).

    Notes
    -----
    The transformation matrix ``H`` is assumed to follow the standard
    homogeneous transformation convention:

    - ``H[:3, :3]`` contains the 3x3 rotation matrix.
    - ``H[:3, 3]`` contains the 3D translation vector.

    """

    id: int
    H: np.ndarray

    @property
    def position(self) -> np.ndarray:
        """Return the 3D position vector.

        Extracted from the transformation matrix (shape: (3,)).
        """
        return self.H[:3, 3]


    @property
    def orientation(self) -> np.ndarray:
        """Return the 3x3 rotation matrix.

        Extracted from the transformation matrix (shape: (3, 3)).
        """
        return self.H[:3, :3]