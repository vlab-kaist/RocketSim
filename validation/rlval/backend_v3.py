"""Replay backend for RocketSim v3 (Rust), through the `replay_v3` command line tool."""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
_BIN = Path(os.environ.get("RLVAL_REPLAY_V3", _ROOT / "replay_v3/target/release/replay_v3"))
_MESHES = Path(os.environ.get("RLVAL_MESHES", _ROOT.parents[1] / "collision_meshes"))


def replay(spec: dict, name: str = "v3") -> dict:
    if not _BIN.exists():
        raise FileNotFoundError(f"{_BIN} not found; run `cargo build --release` in validation/replay_v3")
    proc = subprocess.run([str(_BIN), str(_MESHES)], input=json.dumps(spec), capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"replay_v3 failed: {proc.stderr.strip()}")
    out = json.loads(proc.stdout)
    out["backend"] = name
    return out
