"""Turn a recording into a backend-neutral replay job, and parse backend results.

A job is plain JSON so that backends in any language can run it:

    {"game_mode": "soccar", "gravity_z": -650.0,
     "cars": [{"team": 0, "body": "octane", "pos": [...], "rot": [fwd3, right3, up3],
               "vel": [...], "ang_vel": [...], "boost": 100.0, "on_ground": true,
               "has_jumped": false, "has_double_jumped": false, "has_flipped": false,
               "demoed": false, "supersonic": false, "handbrake": 0.0}],
     "ball": {"pos": [...], "rot": [...], "vel": [...], "ang_vel": [...]},
     "steps": [{"ticks": 1, "controls": [[8 floats] per car]}]}

A result holds the state after every step:

    {"backend": "...", "ball": [[pos3, vel3, ang_vel3] flattened to 9] per step,
     "cars": [[[pos3, fwd3, up3, vel3, ang_vel3, boost, on_ground, supersonic, demoed]
               flattened to 19] per car] per step,
     "events": [{"step": i, "attacker": a, "victim": v, "demo": bool}]}
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .common import AIR_ON_GROUND, TICK_RATE, euler_to_rotmat
from .recording import PHASE_RUN, Recording

CAR_ROW = 19


@dataclass
class Job:
    spec: dict
    start: int  # recording index of the initial state
    indices: np.ndarray  # recording index reached after each step


def _start_from_scenario(rec: Recording, run: np.ndarray) -> int | None:
    """First run packet whose bodies match the state the scenario set, if the scenario is known.

    The game applies a state set a few packets after it was sent, and a body set at
    rest in the air moves too little for a teleport test, so match the set state.
    """
    from .scenarios import select

    found = [s for s in select([rec.meta.get("scenario", "")]) if s.name == rec.meta.get("scenario")]
    if not found:
        return None
    s = found[0]
    want_pos = np.array([c.pos for c in s.cars] + [s.ball.pos], dtype=np.float64)
    want_vel = np.array([c.vel for c in s.cars] + [s.ball.vel], dtype=np.float64)
    for i in run[:12]:
        pos = np.concatenate([rec.car_pos[i, : len(s.cars)], rec.ball_pos[i][None]], axis=0)
        vel = np.concatenate([rec.car_vel[i, : len(s.cars)], rec.ball_vel[i][None]], axis=0)
        # One tick of motion from the set state at most
        if np.all(np.linalg.norm(pos - want_pos, axis=-1) < 2.0 + np.linalg.norm(want_vel, axis=-1) / TICK_RATE) and np.all(
            np.linalg.norm(vel - want_vel, axis=-1) < 15.0
        ):
            return int(i)
    return None


def find_start(rec: Recording) -> int:
    """First run-phase packet after the last state-set teleport."""
    run = np.flatnonzero(rec.phase == PHASE_RUN)
    if len(run) == 0:
        raise ValueError("recording has no run phase")
    if rec.meta.get("settle", 1) == 0:
        found = _start_from_scenario(rec, run)
        if found is not None:
            return found
    start = int(run[0])
    # The state set lands within the first few run packets; later jumps are game events (demo respawns)
    window = run[:6]
    for i in window[:-1]:
        dframes = max(int(rec.frame_num[i + 1] - rec.frame_num[i]), 1)
        bodies_pos = np.concatenate([rec.car_pos[i : i + 2], rec.ball_pos[i : i + 2, None]], axis=1)
        bodies_vel = np.concatenate([rec.car_vel[i], rec.ball_vel[i][None]], axis=0)
        moved = np.linalg.norm(bodies_pos[1] - bodies_pos[0], axis=-1)
        expected = np.linalg.norm(bodies_vel, axis=-1) * dframes / TICK_RATE
        if np.any(moved > expected + 30.0):
            start = int(i) + 1
    return start


def _handbrake_at(rec: Recording, src: np.ndarray, delay: int, end: int, car: int) -> float:
    """Smoothed handbrake value the game has at packet `end`, from the inputs that drove the ticks before it.

    State setting doesn't touch it, and a scenario's first inputs can already
    act on the ticks before the start packet.
    """
    rise, fall = 5.0 / TICK_RATE, 2.0 / TICK_RATE
    val = 0.0
    for i in range(end):
        ticks = int(rec.frame_num[i + 1] - rec.frame_num[i])
        j = min(max(i + delay, 0), len(src) - 1)
        pressed = bool(np.nan_to_num(src[j, car, 7], nan=0.0) > 0.5)
        for _ in range(max(ticks, 0)):
            val = min(val + rise, 1.0) if pressed else max(val - fall, 0.0)
    return val


def _rot_rows(pyr: np.ndarray) -> list[list[float]]:
    m = euler_to_rotmat(pyr)
    return [m[:, 0].tolist(), m[:, 1].tolist(), m[:, 2].tolist()]


def build_job(rec: Recording, delay: int = 0, start: int | None = None, max_steps: int | None = None, inputs: str = "last") -> Job:
    """Build a replay job from the run phase of a recording.

    `delay` shifts which packet's input drives each step: the step from packet i
    to i+1 uses the input reported at packet i + delay (clamped to the recording).
    `inputs` selects the input source: "last" (what the game reports it applied)
    or "sent" (what the recorder sent in reply to each packet).
    """
    start = find_start(rec) if start is None else start
    end = len(rec.frame_num) - 1
    if max_steps is not None:
        end = min(end, start + max_steps)
    src = rec.last_input if inputs == "last" else rec.sent_input

    cars = []
    for c, info in enumerate(rec.meta["cars"]):
        on_ground = int(rec.car_air_state[start, c]) == AIR_ON_GROUND
        cars.append(
            {
                "team": int(info["team"]),
                "body": info["body"],
                "pos": rec.car_pos[start, c].tolist(),
                "rot": _rot_rows(rec.car_rot[start, c]),
                "vel": rec.car_vel[start, c].tolist(),
                "ang_vel": rec.car_ang_vel[start, c].tolist(),
                "boost": float(rec.car_boost[start, c]),
                "on_ground": on_ground,
                "has_jumped": bool(rec.car_has_jumped[start, c]),
                "has_double_jumped": bool(rec.car_has_double_jumped[start, c]),
                "has_flipped": bool(rec.car_has_dodged[start, c]),
                "demoed": bool(rec.car_demolished_timeout[start, c] >= 0),
                "supersonic": bool(rec.car_supersonic[start, c]),
                "handbrake": _handbrake_at(rec, src, delay, start, c),
            }
        )
    ball = {
        "pos": rec.ball_pos[start].tolist(),
        "rot": _rot_rows(rec.ball_rot[start]),
        "vel": rec.ball_vel[start].tolist(),
        "ang_vel": rec.ball_ang_vel[start].tolist(),
    }
    steps, indices = [], []
    for i in range(start, end):
        ticks = int(rec.frame_num[i + 1] - rec.frame_num[i])
        if ticks <= 0:
            continue
        j = min(max(i + delay, 0), len(src) - 1)
        controls = np.nan_to_num(src[j], nan=0.0)
        steps.append({"ticks": ticks, "controls": controls.tolist()})
        indices.append(i + 1)
    spec = {
        "game_mode": _game_mode(rec.meta.get("game_mode", "soccar")),
        "gravity_z": float(rec.meta.get("gravity_z", -650.0)),
        "cars": cars,
        "ball": ball,
        "steps": steps,
    }
    return Job(spec=spec, start=start, indices=np.array(indices, dtype=np.int64))


def _game_mode(name: str) -> str:
    name = name.lower()
    for key, mode in (("hoops", "hoops"), ("dropshot", "dropshot"), ("snow", "snowday"), ("hockey", "snowday"), ("heatseeker", "heatseeker")):
        if key in name:
            return mode
    return "soccar"


@dataclass
class Result:
    backend: str
    ball: np.ndarray  # [S,9]
    cars: np.ndarray  # [S,C,19]
    events: list[dict]

    @staticmethod
    def from_dict(d: dict) -> "Result":
        cars = np.array(d["cars"], dtype=np.float64)
        return Result(d["backend"], np.array(d["ball"], dtype=np.float64), cars.reshape(len(d["ball"]), -1, CAR_ROW), d.get("events", []))
