"""Shared constants and conversions between RLBot packets and simulator state."""

from __future__ import annotations

import numpy as np

TICK_RATE = 120

# Order of the 8 control values everywhere in this package.
CONTROL_NAMES = ("throttle", "steer", "pitch", "yaw", "roll", "jump", "boost", "handbrake")
NUM_CONTROLS = len(CONTROL_NAMES)

# RLBot AirState enum values (flat.AirState).
AIR_ON_GROUND, AIR_JUMPING, AIR_DOUBLE_JUMPING, AIR_DODGING, AIR_IN_AIR = range(5)

# Hitbox (length, width, height) -> body preset name. Values are what RLBot reports.
HITBOXES = {
    "octane": (118.01, 84.20, 36.16),
    "dominus": (127.93, 83.28, 31.30),
    "plank": (128.82, 84.67, 29.39),
    "breakout": (131.49, 80.52, 30.30),
    "hybrid": (127.02, 82.19, 34.16),
    "merc": (120.72, 76.71, 41.66),
}


def body_from_hitbox(length: float, width: float, height: float) -> str:
    """Return the preset whose hitbox is closest to the reported one."""
    dims = np.array([length, width, height])
    return min(HITBOXES, key=lambda k: float(np.abs(np.array(HITBOXES[k]) - dims).sum()))


def euler_to_rotmat(pyr: np.ndarray) -> np.ndarray:
    """RLBot (pitch, yaw, roll) in radians -> 3x3 matrix with columns (forward, right, up).

    Accepts any leading batch shape; the last axis is the 3 angles.
    Same convention as rlgym_compat, which is what RocketSim expects.
    """
    pyr = np.asarray(pyr, dtype=np.float64)
    cp, cy, cr = np.cos(pyr[..., 0]), np.cos(pyr[..., 1]), np.cos(pyr[..., 2])
    sp, sy, sr = np.sin(pyr[..., 0]), np.sin(pyr[..., 1]), np.sin(pyr[..., 2])
    m = np.empty(pyr.shape[:-1] + (3, 3))
    m[..., 0, 0] = cp * cy
    m[..., 1, 0] = cp * sy
    m[..., 2, 0] = sp
    m[..., 0, 1] = cy * sp * sr - cr * sy
    m[..., 1, 1] = sy * sp * sr + cr * cy
    m[..., 2, 1] = -cp * sr
    m[..., 0, 2] = -cr * cy * sp - sr * sy
    m[..., 1, 2] = -cr * sy * sp + sr * cy
    m[..., 2, 2] = cp * cr
    return m


def rotmat_angle_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Angle in degrees of the rotation taking matrix a to matrix b (batched)."""
    rel = np.swapaxes(a, -1, -2) @ b
    cos = (np.trace(rel, axis1=-2, axis2=-1) - 1.0) / 2.0
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))
