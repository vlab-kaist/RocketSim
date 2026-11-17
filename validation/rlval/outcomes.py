"""Print what each backend does with the car-car scenarios: demo, bump, or nothing.

    python -m rlval.outcomes --backend cpp --label cpp-python-dev --out outcomes/
    python -m rlval.outcomes --merge outcomes/*.json

Runs the scenarios straight from their definitions, so no recording is needed.
The first form writes one JSON per backend build; the second prints them side by side.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from . import backends
from .job import Result
from .scenarios import select
from .synth import scenario_spec


def run(backend: str, patterns: list[str]) -> dict:
    out = {}
    for s in select(patterns or ["car_car"]):
        res = Result.from_dict(backends.get(backend)(scenario_spec(s)))
        first = res.events[0] if res.events else None
        attacker_speed = None
        if first is not None:
            k = max(first["step"] - 1, 0)
            attacker_speed = float(np.linalg.norm(res.cars[k, first["attacker"], 9:12]))
        out[s.name] = {
            "outcome": "none" if first is None else ("DEMO" if any(e["demo"] for e in res.events) else "bump"),
            "events": len(res.events),
            "attacker": None if first is None else first["attacker"],
            "attacker_speed": attacker_speed,
            "victim_speed_after": float(np.linalg.norm(res.cars[-1, 1, 9:12])) if first else 0.0,
        }
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("patterns", nargs="*")
    ap.add_argument("--backend", choices=["cpp", "v3"])
    ap.add_argument("--label")
    ap.add_argument("--out", type=Path, default=Path("outcomes"))
    ap.add_argument("--merge", nargs="+", type=Path)
    args = ap.parse_args()

    if args.merge:
        tables = {p.stem: json.loads(p.read_text()) for p in args.merge}
        labels = list(tables)
        names = list(next(iter(tables.values())))
        print(f"{'scenario':28s} " + " ".join(f"{label:>22s}" for label in labels))
        for n in names:
            cells = []
            for label in labels:
                r = tables[label].get(n)
                if r is None:
                    cells.append("-")
                elif r["outcome"] == "none":
                    cells.append("none")
                else:
                    who = "" if r["attacker"] == 0 else f" by car{r['attacker']}"
                    cells.append(f"{r['outcome']}{who} @{r['attacker_speed']:.0f}")
            differs = len({c.split(" @")[0] for c in cells}) > 1
            print(f"{n:28s} " + " ".join(f"{c:>22s}" for c in cells) + ("   <<" if differs else ""))
        return

    if not args.backend:
        ap.error("--backend or --merge is required")
    label = args.label or args.backend
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / f"{label}.json").write_text(json.dumps(run(args.backend, args.patterns), indent=1))
    print(f"wrote {args.out / (label + '.json')}")


if __name__ == "__main__":
    main()
