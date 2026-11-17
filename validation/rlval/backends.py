"""Backend registry. Each backend maps a replay job spec to a result dict."""

from __future__ import annotations

from typing import Callable


def get(name: str) -> Callable[[dict], dict]:
    if name == "v3":
        from . import backend_v3

        return backend_v3.replay
    if name == "cpp":
        from . import backend_cpp

        return backend_cpp.replay
    raise ValueError(f"unknown backend {name!r}; expected 'cpp' or 'v3'")
