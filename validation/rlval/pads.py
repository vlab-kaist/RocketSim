"""Boost pad pickups: the tick a car's boost goes up in the real game against the simulator.

    python -m rlval.pads recordings/real-2026-11-17 recordings/real-2026-11-19-random [--backends cpp v3] [-v]

Recordings are replayed as in rlval.compare (last_input, --delay 1), starting from the recorded pad
cooldowns. A pickup is a step where a car's boost rises (respawns after a demo excluded). Each game pickup
is matched to the same car's nearest simulator pickup within 60 ticks; reported are the tick differences
(simulator - game; negative = the simulator picks it up earlier), game pickups the simulator missed and
simulator pickups the game didn't have.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from . import backends
from .job import Result, build_job
from .recording import Recording

WINDOW = 60
DEMOED = 18  # Result car row: is_demoed
BOOST = 15


def _pickups(boost: np.ndarray, demoed: np.ndarray) -> list[int]:
    """Steps where the boost rose, not around a demo respawn."""
    out = []
    for s in range(1, len(boost)):
        if boost[s] > boost[s - 1] + 0.5 and not demoed[max(s - 3, 0):s + 1].any():
            out.append(s)
    return out


def _match(game: list[int], sim: list[int]) -> tuple[list[int], int, int]:
    diffs, used = [], set()
    for g in game:
        cand = [s for s in sim if s not in used and abs(s - g) <= WINDOW]
        if cand:
            s = min(cand, key=lambda s: abs(s - g))
            used.add(s)
            diffs.append(s - g)
    return diffs, len(game) - len(diffs), len(sim) - len(used)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", type=Path)
    ap.add_argument("--backends", nargs="+", default=["cpp"])
    ap.add_argument("--delay", type=int, default=1)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    files = sorted(f for p in args.paths for f in ([p] if p.is_file() else p.glob("*.npz")))
    total = {b: {"diffs": [], "missed": 0, "extra": 0} for b in args.backends}
    n_game = 0
    for f in files:
        rec = Recording.load(f)
        if rec.pad_active is None or not len(rec.pad_active):
            continue
        job = build_job(rec, delay=args.delay, inputs="last")
        idx = job.indices
        if not len(idx):
            continue
        rows = np.concatenate([[job.start], idx])
        game_boost = rec.car_boost[rows]
        game_demoed = rec.car_demolished_timeout[rows] >= 0
        game = [_pickups(game_boost[:, c], game_demoed[:, c]) for c in range(rec.num_cars)]
        if not any(game):
            continue
        n_game += sum(len(g) for g in game)
        for b in args.backends:
            res = Result.from_dict(backends.get(b)(job.spec))
            boost = np.concatenate([game_boost[:1], res.cars[:, :, BOOST]])
            demoed = np.concatenate([game_demoed[:1], res.cars[:, :, DEMOED] > 0.5])
            for c in range(rec.num_cars):
                sim = _pickups(boost[:, c], demoed[:, c])
                d, missed, extra = _match(game[c], sim)
                t = total[b]
                t["diffs"] += d
                t["missed"] += missed
                t["extra"] += extra
                if args.verbose and (missed or extra or any(d)):
                    print(f"{f.parent.name}/{f.name:34s} {b:4s} car {c}: game {game[c]} sim {sim} diffs {d}")
    print(f"{n_game} game pickups")
    for b, t in total.items():
        d = np.array(t["diffs"])
        counts = dict(zip(*(x.tolist() for x in np.unique(d, return_counts=True)))) if len(d) else {}
        print(f"{b}: same tick {int((d == 0).sum())}/{n_game}, tick differences {counts}, missed {t['missed']}, extra {t['extra']}")


if __name__ == "__main__":
    main()
