"""Goal timing: the tick the real game counted a goal against the tick the simulator does.

    python -m rlval.goals recordings/real-2026-11-19-goal [--backends cpp]

Only recordings that ended with a goal (recorded with goal_run_tick) are used. The game's tick is the run
tick of the first packet that shows the new score; the simulator's is the first replay step whose tick
scored (Arena goal score callback), as a run tick. Both are printed with the ball's y and z there.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from . import backends
from .job import Result, build_job
from .recording import Recording


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", type=Path)
    ap.add_argument("--backends", nargs="+", default=["cpp"])
    ap.add_argument("--delay", type=int, default=1)
    args = ap.parse_args()

    files = sorted(f for p in args.paths for f in ([p] if p.is_file() else p.glob("*.npz")))
    print(f"{'scenario':34s} {'backend':7s} {'game tick':>9s} {'sim tick':>8s} {'diff':>5s}  ball y/z at the game's goal tick (rec)")
    diffs = {b: [] for b in args.backends}
    for f in files:
        rec = Recording.load(f)
        if rec.meta.get("ended_by") != "goal" or "goal_run_tick" not in rec.meta:
            continue
        game_tick = int(rec.meta["goal_run_tick"])
        # Replay a bit past the recorded end so the simulator can reach the goal
        job = build_job(rec, delay=args.delay, max_steps=None, inputs="last", extra_steps=60)
        start_tick = int(rec.run_tick[job.start])
        last = rec.ball_pos[-1]
        for b in args.backends:
            res = Result.from_dict(backends.get(b)(job.spec))
            goal = [e for e in res.events if e.get("goal")]
            # (Run tick after the step the goal was scored in)
            sim_tick = start_tick + sum(s["ticks"] for s in job.spec["steps"][: goal[0]["step"] + 1]) if goal else None
            diff = (sim_tick - game_tick) if sim_tick is not None else None
            diffs[b].append(diff)
            print(f"{rec.meta['scenario']:34s} {b:7s} {game_tick:9d} {str(sim_tick):>8s} {str(diff):>5s}  last recorded ball y {last[1]:8.2f} z {last[2]:7.2f}")
    for b, d in diffs.items():
        known = [x for x in d if x is not None]
        print(f"{b}: {len(d)} goals, sim missed {d.count(None)}, tick difference counts {dict(zip(*np.unique(known, return_counts=True))) if known else {}}")


if __name__ == "__main__":
    main()
