# LiDAR reader: returns raw point clouds in LiDAR frame.

from typing import Optional
import numpy as np


def get_point_cloud() -> Optional[np.ndarray]:
    """Return an (N, 3) NumPy array of XYZ points in LiDAR frame.

    For now this is a stub. Replace with actual LiDAR packet parsing.
    """
    # TODO: Implement LiDAR UDP reading and parsing
    return None
