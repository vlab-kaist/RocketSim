"""On-disk recording format: one .npz per scenario run.

Arrays are indexed by recorded packet (T) and car (C). Every packet the server
sends is stored, so `frame_num` shows directly whether any game tick was missed.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .common import NUM_CONTROLS

PHASE_SETUP, PHASE_SETTLE, PHASE_RUN = 0, 1, 2


@dataclass
class Recording:
    meta: dict
    frame_num: np.ndarray  # [T] game physics frame
    seconds: np.ndarray  # [T] MatchInfo.seconds_elapsed
    phase: np.ndarray  # [T] PHASE_*
    run_tick: np.ndarray  # [T] tick index inside the run phase, -1 outside
    ball_pos: np.ndarray  # [T,3]
    ball_rot: np.ndarray  # [T,3] pitch, yaw, roll
    ball_vel: np.ndarray  # [T,3]
    ball_ang_vel: np.ndarray  # [T,3]
    car_pos: np.ndarray  # [T,C,3]
    car_rot: np.ndarray  # [T,C,3] pitch, yaw, roll
    car_vel: np.ndarray  # [T,C,3]
    car_ang_vel: np.ndarray  # [T,C,3]
    car_boost: np.ndarray  # [T,C] 0..100
    car_air_state: np.ndarray  # [T,C] flat.AirState
    car_supersonic: np.ndarray  # [T,C] bool
    car_has_jumped: np.ndarray  # [T,C] bool
    car_has_double_jumped: np.ndarray  # [T,C] bool
    car_has_dodged: np.ndarray  # [T,C] bool
    car_dodge_timeout: np.ndarray  # [T,C]
    car_dodge_elapsed: np.ndarray  # [T,C]
    car_dodge_dir: np.ndarray  # [T,C,2]
    car_demolished_timeout: np.ndarray  # [T,C] -1 when alive
    car_touch_seconds: np.ndarray  # [T,C] game_seconds of latest touch, -1 if none
    last_input: np.ndarray  # [T,C,8] input the game reports as applied
    sent_input: np.ndarray  # [T,C,8] input sent in reply to this packet, NaN if none
    pad_active: np.ndarray = field(default_factory=lambda: np.zeros((0, 0), bool))  # [T,P]
    pad_timer: np.ndarray = field(default_factory=lambda: np.zeros((0, 0), np.float32))  # [T,P]

    @property
    def num_cars(self) -> int:
        return self.car_pos.shape[1]

    def save(self, path: Path) -> None:
        arrays = {k: v for k, v in self.__dict__.items() if k != "meta"}
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, meta=json.dumps(self.meta), **arrays)

    @staticmethod
    def load(path: Path) -> "Recording":
        with np.load(path, allow_pickle=False) as z:
            meta = json.loads(str(z["meta"]))
            return Recording(meta=meta, **{k: z[k] for k in z.files if k != "meta"})


class RecordingBuilder:
    """Accumulates per-packet rows and turns them into a Recording."""

    def __init__(self, meta: dict, num_cars: int):
        self.meta = meta
        self.num_cars = num_cars
        self.rows: list[dict] = []

    def add(self, row: dict) -> int:
        self.rows.append(row)
        return len(self.rows) - 1

    def set_sent(self, index: int, controls: list[list[float]]) -> None:
        self.rows[index]["sent_input"] = controls

    def build(self) -> Recording:
        nan_inputs = [[float("nan")] * NUM_CONTROLS] * self.num_cars
        for row in self.rows:
            row.setdefault("sent_input", nan_inputs)
        keys = self.rows[0].keys()
        ints = {"frame_num": np.int64, "phase": np.int8, "run_tick": np.int32, "car_air_state": np.int8}
        bools = {"car_supersonic", "car_has_jumped", "car_has_double_jumped", "car_has_dodged", "pad_active"}
        arrays = {}
        for k in keys:
            dtype = ints.get(k, np.bool_ if k in bools else np.float64 if k == "seconds" else np.float32)
            arrays[k] = np.array([r[k] for r in self.rows], dtype=dtype)
        return Recording(meta=self.meta, **arrays)
