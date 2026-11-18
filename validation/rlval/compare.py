"""Replay recordings in simulator backends and report how far they drift.

    python -m rlval.compare recordings/ --backends cpp v3 [--fit-delay] [--csv out.csv]

Each recording is replayed open loop from the start of its run phase: the
simulator gets the recorded initial state and the recorded inputs, and its
trajectory is compared with the recorded one at fixed horizons.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import backends
from .common import TICK_RATE, euler_to_rotmat, rotmat_angle_deg
from .job import Result, build_job
from .recording import Recording

HORIZONS = (0.25, 0.5, 1.0, 2.0)  # seconds after the start of the run


@dataclass
class Score:
    scenario: str
    backend: str
    delay: int
    steps: int
    car_pos: dict[float, float]  # horizon -> max over cars of position error (uu)
    car_pos_end: float
    car_vel_max: float  # max over time and cars of velocity error (uu/s)
    car_rot_max: float  # max over time and cars of orientation error (deg)
    ball_pos_end: float
    car_pos_mean: float  # mean over time and cars, used to pick the input delay
    rec_demo: bool
    sim_demo: bool
    sim_bumps: int
    first_diverge: float  # seconds until any car is more than 10 uu off, inf if never


def _sim_rotmat(cars: np.ndarray) -> np.ndarray:
    fwd, up = cars[..., 3:6], cars[..., 6:9]
    right = np.cross(up, fwd)
    return np.stack([fwd, right, up], axis=-1)


def score(rec: Recording, backend: str, delay: int, inputs: str, max_seconds: float | None) -> Score:
    max_steps = None if max_seconds is None else int(max_seconds * TICK_RATE)
    job = build_job(rec, delay=delay, max_steps=max_steps, inputs=inputs)
    res = Result.from_dict(backends.get(backend)(job.spec))
    idx = job.indices
    t = (rec.frame_num[idx] - rec.frame_num[job.start]) / TICK_RATE

    # A demolished car is hidden by the game and parked by the simulator, so its
    # physics is not comparable.
    alive = (rec.car_demolished_timeout[idx] < 0) & (res.cars[..., 18] < 0.5)
    pos_err = np.where(alive, np.linalg.norm(res.cars[..., 0:3] - rec.car_pos[idx], axis=-1), 0.0)
    vel_err = np.where(alive, np.linalg.norm(res.cars[..., 9:12] - rec.car_vel[idx], axis=-1), 0.0)
    rot_err = np.where(alive, rotmat_angle_deg(_sim_rotmat(res.cars), euler_to_rotmat(rec.car_rot[idx])), 0.0)
    ball_err = np.linalg.norm(res.ball[:, 0:3] - rec.ball_pos[idx], axis=-1)

    worst = pos_err.max(axis=1)
    at = {}
    for h in HORIZONS:
        k = int(np.searchsorted(t, h))
        at[h] = float(worst[k]) if k < len(worst) else float("nan")
    off = np.flatnonzero(worst > 10.0)
    return Score(
        scenario=rec.meta["scenario"],
        backend=backend,
        delay=delay,
        steps=len(idx),
        car_pos=at,
        car_pos_end=float(worst[-1]),
        car_vel_max=float(vel_err.max()),
        car_rot_max=float(rot_err.max()),
        ball_pos_end=float(ball_err[-1]),
        car_pos_mean=float(pos_err.mean()),
        rec_demo=bool((rec.car_demolished_timeout[idx] >= 0).any()),
        sim_demo=any(e["demo"] for e in res.events),
        sim_bumps=sum(not e["demo"] and not e.get("goal") for e in res.events),
        first_diverge=float(t[off[0]]) if len(off) else float("inf"),
    )


def best_delay(rec: Recording, backend: str, inputs: str, candidates: range, max_seconds: float | None) -> Score:
    scores = [score(rec, backend, d, inputs, max_seconds) for d in candidates]
    return min(scores, key=lambda s: s.car_pos_mean)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", type=Path, help="recording files or directories")
    ap.add_argument("--backends", nargs="+", default=["cpp", "v3"])
    ap.add_argument("--delay", type=int, default=1, help="input delay in ticks (see job.build_job)")
    ap.add_argument("--fit-delay", action="store_true", help="try delays -1..4 and keep the best per recording")
    ap.add_argument("--inputs", choices=["last", "sent"], default="last")
    ap.add_argument("--max-seconds", type=float, default=None, help="only replay this much of each run")
    ap.add_argument("--csv", type=Path, default=None)
    args = ap.parse_args()

    files = sorted(f for p in args.paths for f in ([p] if p.is_file() else p.glob("*.npz")))
    if not files:
        raise SystemExit("no recordings found")

    header = f"{'scenario':34s} {'backend':7s} {'dly':>3s} " + " ".join(f"{f'pos@{h}s':>9s}" for h in HORIZONS)
    header += f" {'pos@end':>9s} {'velmax':>8s} {'rotmax':>7s} {'ball@end':>9s} {'diverge':>8s}  demo(rec/sim) bumps"
    print(header)
    rows = []
    skipped = []
    for f in files:
        rec = Recording.load(f)
        run = np.flatnonzero(rec.run_tick >= 0)
        if len(run) and (rec.car_demolished_timeout[: run[0] + 1] >= 0).any():
            # Recorded before the recorder waited for respawns: a car demolished in the previous run was
            # still missing, so the scenario didn't happen as set up
            skipped.append(f.name)
            continue
        for b in args.backends:
            if args.fit_delay:
                s = best_delay(rec, b, args.inputs, range(-1, 5), args.max_seconds)
            else:
                s = score(rec, b, args.delay, args.inputs, args.max_seconds)
            rows.append(s)
            div = f"{s.first_diverge:7.2f}s" if np.isfinite(s.first_diverge) else "   never"
            flag = "" if s.rec_demo == s.sim_demo else "  << demo mismatch"
            print(
                f"{s.scenario:34s} {s.backend:7s} {s.delay:3d} "
                + " ".join(f"{s.car_pos[h]:9.2f}" for h in HORIZONS)
                + f" {s.car_pos_end:9.2f} {s.car_vel_max:8.1f} {s.car_rot_max:7.2f} {s.ball_pos_end:9.2f} {div}  "
                + f"{int(s.rec_demo)}/{int(s.sim_demo)}           {s.sim_bumps}{flag}"
            )

    print()
    if skipped:
        print(f"skipped {len(skipped)} recordings that start with a demolished car: {' '.join(skipped)}")
    for b in args.backends:
        mine = [s for s in rows if s.backend == b]
        mismatches = sum(s.rec_demo != s.sim_demo for s in mine)
        never = sum(not np.isfinite(s.first_diverge) for s in mine)
        med = np.nanmedian([s.car_pos[1.0] for s in mine])
        print(f"{b}: {len(mine)} recordings, median car pos error at 1s {med:.2f} uu, {never} never diverge past 10 uu, {mismatches} demo mismatches")

    if args.csv:
        with args.csv.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["scenario", "backend", "delay", "steps"] + [f"pos_{h}s" for h in HORIZONS] + ["pos_end", "vel_max", "rot_max_deg", "ball_pos_end", "first_diverge_s", "rec_demo", "sim_demo", "sim_bumps"])
            for s in rows:
                w.writerow([s.scenario, s.backend, s.delay, s.steps] + [s.car_pos[h] for h in HORIZONS] + [s.car_pos_end, s.car_vel_max, s.car_rot_max, s.ball_pos_end, s.first_diverge, int(s.rec_demo), int(s.sim_demo), s.sim_bumps])


if __name__ == "__main__":
    main()
