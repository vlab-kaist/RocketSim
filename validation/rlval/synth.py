"""Generate recordings from a simulator instead of the game.

This exercises the whole pipeline without Rocket League and gives a direct
backend-vs-backend comparison on the scenario suite:

    python -m rlval.synth --backend cpp --out recordings_synth/cpp [patterns...]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from . import backends
from .common import AIR_IN_AIR, AIR_ON_GROUND, NUM_CONTROLS, euler_to_rotmat
from .job import Result
from .recording import PHASE_RUN, PHASE_SETTLE, Recording
from .scenarios import Body, Scenario, select


def _rot_rows(b: Body) -> list[list[float]]:
    m = euler_to_rotmat(np.array([b.pitch, b.yaw, b.roll]))
    return [m[:, 0].tolist(), m[:, 1].tolist(), m[:, 2].tolist()]


def _euler(fwd: np.ndarray, up: np.ndarray) -> np.ndarray:
    """(forward, up) columns -> RLBot (pitch, yaw, roll), batched over leading axes."""
    right = np.cross(up, fwd)
    pitch = np.arcsin(np.clip(fwd[..., 2], -1.0, 1.0))
    yaw = np.arctan2(fwd[..., 1], fwd[..., 0])
    roll = np.arctan2(-right[..., 2], up[..., 2])
    return np.stack([pitch, yaw, roll], axis=-1)


def scenario_spec(s: Scenario) -> dict:
    cars = [
        {
            "team": i % 2,
            "body": "octane",
            "pos": list(c.pos),
            "rot": _rot_rows(c),
            "vel": list(c.vel),
            "ang_vel": list(c.ang_vel),
            "boost": c.boost,
            "on_ground": c.pos[2] < 20.0,
            "has_jumped": False,
            "has_double_jumped": False,
            "has_flipped": False,
            "demoed": False,
            "supersonic": float(np.linalg.norm(c.vel)) >= 2200.0,
        }
        for i, c in enumerate(s.cars)
    ]
    ball = {"pos": list(s.ball.pos), "rot": _rot_rows(s.ball), "vel": list(s.ball.vel), "ang_vel": list(s.ball.ang_vel)}
    steps = [{"ticks": 1, "controls": s.controls_at(t)} for t in range(-s.settle, s.duration)]
    return {"game_mode": "soccar", "gravity_z": -650.0, "cars": cars, "ball": ball, "steps": steps}


def synthesize(s: Scenario, backend: str) -> Recording:
    spec = scenario_spec(s)
    res = Result.from_dict(backends.get(backend)(spec))
    n, c = len(spec["steps"]) + 1, len(s.cars)

    def with_initial(initial, rest):
        return np.concatenate([np.asarray(initial, dtype=np.float64)[None], rest], axis=0)

    car0 = spec["cars"]
    pos = with_initial([x["pos"] for x in car0], res.cars[:, :, 0:3])
    fwd = with_initial([x["rot"][0] for x in car0], res.cars[:, :, 3:6])
    up = with_initial([x["rot"][2] for x in car0], res.cars[:, :, 6:9])
    vel = with_initial([x["vel"] for x in car0], res.cars[:, :, 9:12])
    ang = with_initial([x["ang_vel"] for x in car0], res.cars[:, :, 12:15])
    boost = with_initial([x["boost"] for x in car0], res.cars[:, :, 15])
    ground = with_initial([x["on_ground"] for x in car0], res.cars[:, :, 16]) > 0.5
    supersonic = with_initial([x["supersonic"] for x in car0], res.cars[:, :, 17]) > 0.5
    demoed = with_initial([0.0] * c, res.cars[:, :, 18]) > 0.5
    b0 = spec["ball"]
    controls = np.array([st["controls"] for st in spec["steps"]], dtype=np.float32)  # [n-1,C,8]
    sent = np.concatenate([controls, np.full((1, c, NUM_CONTROLS), np.nan, np.float32)], axis=0)
    # The game reports at packet i the input it applied to reach packet i.
    last = np.concatenate([np.zeros((1, c, NUM_CONTROLS), np.float32), controls], axis=0)
    run_tick = np.arange(n) - s.settle
    zeros = np.zeros((n, c), np.float32)
    return Recording(
        meta={
            "scenario": s.name,
            "repeat": 0,
            "tags": s.tags,
            "source": f"synth:{backend}",
            "game_mode": "soccar",
            "gravity_z": -650.0,
            "game_speed": 1.0,
            "cars": [{"name": f"car{i}", "team": i % 2, "body": "octane"} for i in range(c)],
            "settle": s.settle,
            "duration": s.duration,
            "dropped_frames": 0,
        },
        frame_num=np.arange(n, dtype=np.int64),
        seconds=np.arange(n) / 120.0,
        phase=np.where(run_tick >= 0, PHASE_RUN, PHASE_SETTLE).astype(np.int8),
        run_tick=np.where(run_tick >= 0, run_tick, -1).astype(np.int32),
        ball_pos=with_initial(b0["pos"], res.ball[:, 0:3]).astype(np.float32),
        ball_rot=np.zeros((n, 3), np.float32),
        ball_vel=with_initial(b0["vel"], res.ball[:, 3:6]).astype(np.float32),
        ball_ang_vel=with_initial(b0["ang_vel"], res.ball[:, 6:9]).astype(np.float32),
        car_pos=pos.astype(np.float32),
        car_rot=_euler(fwd, up).astype(np.float32),
        car_vel=vel.astype(np.float32),
        car_ang_vel=ang.astype(np.float32),
        car_boost=boost.astype(np.float32),
        car_air_state=np.where(ground, AIR_ON_GROUND, AIR_IN_AIR).astype(np.int8),
        car_supersonic=supersonic,
        car_has_jumped=np.zeros((n, c), bool),
        car_has_double_jumped=np.zeros((n, c), bool),
        car_has_dodged=np.zeros((n, c), bool),
        car_dodge_timeout=zeros - 1,
        car_dodge_elapsed=zeros,
        car_dodge_dir=np.zeros((n, c, 2), np.float32),
        car_demolished_timeout=np.where(demoed, 3.0, -1.0).astype(np.float32),
        car_touch_seconds=zeros - 1,
        last_input=last,
        sent_input=sent,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("patterns", nargs="*")
    ap.add_argument("--backend", required=True, choices=["cpp", "v3"])
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    scenarios = select(args.patterns)
    for s in scenarios:
        synthesize(s, args.backend).save(args.out / f"{s.name}__r0.npz")
    print(f"wrote {len(scenarios)} recordings to {args.out}")


if __name__ == "__main__":
    main()
