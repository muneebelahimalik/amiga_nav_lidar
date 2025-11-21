# Safety layer that checks LiDAR data and adjusts commanded velocities.

from typing import Tuple, Optional


def apply_safety(v_des: float, w_des: float, min_distance_ahead: Optional[float],
                 safe_distance: float = 1.5) -> Tuple[float, float]:
    """Modify desired velocities based on obstacle distance ahead."""
    v_cmd = v_des
    if min_distance_ahead is not None and min_distance_ahead < safe_distance and v_des > 0:
        v_cmd = 0.0
    return v_cmd, w_des
