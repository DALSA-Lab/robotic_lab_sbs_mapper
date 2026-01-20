from dataclasses import dataclass
import numpy as np

@dataclass
class Marker:
    id: int
    H: np.ndarray

    @property
    def position(self):
        return self.H[:3, 3]

    @property
    def orientation(self):
        return self.H[:3, :3]
