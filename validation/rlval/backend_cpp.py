"""Replay backend for the C++ RocketSim fork, through its `RocketSim` Python module.

The module installed in the current environment decides which fork branch is
tested, so compare branches by running this from separate environments.
"""

from __future__ import annotations

import os
from pathlib import Path

import RocketSim as rs

_MESHES = Path(os.environ.get("RLVAL_MESHES", Path(__file__).resolve().parents[3] / "collision_meshes"))
_initialized = False

_MODES = {
    "soccar": rs.GameMode.SOCCAR,
    "hoops": rs.GameMode.HOOPS,
    "heatseeker": rs.GameMode.HEATSEEKER,
    "snowday": rs.GameMode.SNOWDAY,
    "dropshot": rs.GameMode.DROPSHOT,
}
_BODIES = {
    "octane": rs.CarConfig.OCTANE,
    "dominus": rs.CarConfig.DOMINUS,
    "plank": rs.CarConfig.PLANK,
    "breakout": rs.CarConfig.BREAKOUT,
    "hybrid": rs.CarConfig.HYBRID,
    "merc": rs.CarConfig.MERC,
}


def _init() -> None:
    global _initialized
    if not _initialized:
        rs.init(str(_MESHES))
        _initialized = True


def _rotmat(rows) -> "rs.RotMat":
    return rs.RotMat(forward=rs.Vec(*rows[0]), right=rs.Vec(*rows[1]), up=rs.Vec(*rows[2]))


def _vec3(v) -> list[float]:
    return [v.x, v.y, v.z]


def replay(spec: dict, name: str = "cpp") -> dict:
    _init()
    arena = rs.Arena(_MODES[spec["game_mode"]])
    if spec["gravity_z"] != -650.0:
        mc = arena.get_mutator_config()
        mc.gravity = rs.Vec(0, 0, spec["gravity_z"])
        arena.set_mutator_config(mc)

    cars = []
    for c in spec["cars"]:
        car = arena.add_car(rs.Team.BLUE if c["team"] == 0 else rs.Team.ORANGE, rs.CarConfig(_BODIES[c["body"]]))
        st = car.get_state()
        st.pos, st.vel, st.ang_vel = rs.Vec(*c["pos"]), rs.Vec(*c["vel"]), rs.Vec(*c["ang_vel"])
        st.rot_mat = _rotmat(c["rot"])
        st.boost = c["boost"]
        st.is_on_ground = c["on_ground"]
        st.has_jumped = c["has_jumped"]
        st.has_double_jumped = c["has_double_jumped"]
        st.has_flipped = c["has_flipped"]
        st.is_supersonic = c["supersonic"]
        st.handbrake_val = c.get("handbrake", 0.0)
        car.set_state(st)
        if c["demoed"]:
            car.demolish()
        cars.append(car)
    index_of = {car.id: i for i, car in enumerate(cars)}

    b = spec["ball"]
    bs = arena.ball.get_state()
    bs.pos, bs.vel, bs.ang_vel = rs.Vec(*b["pos"]), rs.Vec(*b["vel"]), rs.Vec(*b["ang_vel"])
    bs.rot_mat = _rotmat(b["rot"])
    arena.ball.set_state(bs)

    if spec.get("pads"):
        # The spec lists the pads in RLBot order (sorted by y, then x), which swaps two pairs against RocketSim's
        sim_pads = arena.get_boost_pads()
        rlbot_order = sorted(range(len(sim_pads)), key=lambda j: (sim_pads[j].get_pos().y, sim_pads[j].get_pos().x))
        for cooldown, j in zip(spec["pads"], rlbot_order):
            if cooldown > 0:
                ps = sim_pads[j].get_state()
                ps.is_active, ps.cooldown = False, float(cooldown)
                sim_pads[j].set_state(ps)

    events: list[dict] = []
    step_index = [0]

    def on_bump(arena, bumper, victim, is_demo, data):
        events.append({"step": step_index[0], "attacker": index_of[bumper.id], "victim": index_of[victim.id], "demo": bool(is_demo)})

    arena.set_car_bump_callback(on_bump)

    def on_goal(arena=None, team=None, data=None, **kw):
        # (Called every tick the ball is in a goal; keep the first)
        if not any(e.get("goal") for e in events):
            events.append({"step": step_index[0], "goal": True, "demo": False, "team": int(team)})

    arena.set_goal_score_callback(on_goal)

    ball_rows, car_rows = [], []
    for i, step in enumerate(spec["steps"]):
        step_index[0] = i
        for car, v in zip(cars, step["controls"]):
            car.set_controls(
                rs.CarControls(throttle=v[0], steer=v[1], pitch=v[2], yaw=v[3], roll=v[4], jump=v[5] > 0.5, boost=v[6] > 0.5, handbrake=v[7] > 0.5)
            )
        arena.step(step["ticks"])
        bs = arena.ball.get_state()
        ball_rows.append(_vec3(bs.pos) + _vec3(bs.vel) + _vec3(bs.ang_vel))
        row = []
        for car in cars:
            st = car.get_state()
            row.append(
                _vec3(st.pos)
                + _vec3(st.rot_mat.forward)
                + _vec3(st.rot_mat.up)
                + _vec3(st.vel)
                + _vec3(st.ang_vel)
                + [st.boost, float(st.is_on_ground), float(st.is_supersonic), float(st.is_demoed)]
            )
        car_rows.append(row)
    return {"backend": name, "ball": ball_rows, "cars": car_rows, "events": events}
