"""Replays the planner's real-game chain recordings (rocket-league-planner experiments/rlbot_chain.py,
results/rlbot_loop/*.npz) open loop in the C++ fork and compares them with the game.

    python -m rlval.chain ../../rocket-league-planner/results/rlbot_loop [--delay 1] [-v]

The records hold a packet per tick: ball [T, 9] (pos, vel, ang_vel), cars [T, 2, 35] (rlrl car rows),
last_input [T, 2, 8] (the input the game reports it applied), frame (frame_num), pads [T, 34] (boost pads
active), touch [2] (the game's touch ticks of the two hits). Records without last_input (the planner's
older ones) are skipped. The step from packet t to t+1 uses last_input[t + delay] (1, as in rlval.job).

Start: the cars and ball of packet 0. Boost pads inactive at packet 0 get the cooldown that reactivates
them on the tick the record shows them active again (the full respawn time if it never does).
Reported per record: car position error (max over both cars) before and at the game's first touch and
at the second, the first tick a car is off by more than 2 uu, ball error at the touches, the simulator's
touch ticks against the game's (0 = same tick), boost differences and whether the goal outcome matches.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import RocketSim as rs

from .backend_cpp import _init, _rotmat

# rlrl car row (rlrl/layout.py)
CAR_FORWARD, CAR_BOOST, CAR_ON_GROUND, CAR_HAS_JUMPED, CAR_HAS_DOUBLE_JUMPED, CAR_HAS_FLIPPED, CAR_IS_SUPERSONIC = 9, 18, 19, 20, 21, 22, 27
PAD_RESPAWN = {False: 4.0, True: 10.0}
TICK = 1 / 120


def _set_car(car, row) -> None:
    st = car.get_state()
    st.pos, st.vel, st.ang_vel = rs.Vec(*row[0:3]), rs.Vec(*row[3:6]), rs.Vec(*row[6:9])
    rot = np.asarray(row[CAR_FORWARD:CAR_FORWARD + 9], np.float64).reshape(3, 3)
    st.rot_mat = _rotmat(np.where(np.abs(rot) < 1e-6, 0.0, rot).tolist())
    st.boost = float(row[CAR_BOOST])
    st.is_on_ground = bool(row[CAR_ON_GROUND])
    st.has_jumped = bool(row[CAR_HAS_JUMPED])
    st.has_double_jumped = bool(row[CAR_HAS_DOUBLE_JUMPED])
    st.has_flipped = bool(row[CAR_HAS_FLIPPED])
    st.is_supersonic = bool(row[CAR_IS_SUPERSONIC])
    car.set_state(st)


def _set_pads(arena, pads: np.ndarray) -> None:
    sim_pads = arena.get_boost_pads()
    # The packet lists the pads sorted by (y, x); RocketSim's order swaps the pads at y +-1024 and +-1036
    # (checked against the field info stored in rlval recordings)
    packet_order = sorted(range(len(sim_pads)), key=lambda j: (sim_pads[j].get_pos().y, sim_pads[j].get_pos().x))
    for i, j in enumerate(packet_order):
        pad = sim_pads[j]
        if pads[0, i]:
            continue
        back = np.nonzero(pads[:, i])[0]
        # Active on packet k: the cooldown runs out in the step before it
        cooldown = (back[0] - 0.5) * TICK if len(back) else PAD_RESPAWN[bool(pad.is_big)]
        st = pad.get_state()
        st.is_active, st.cooldown = False, float(cooldown)
        pad.set_state(st)


def replay(d: dict, delay: int = 1) -> dict:
    """Replays a record; returns the simulator's car/ball states, boost, touch ticks and goal."""
    _init()
    arena = rs.Arena(rs.GameMode.SOCCAR)
    cars = [arena.add_car(rs.Team.BLUE, rs.CarConfig(rs.CarConfig.OCTANE)) for _ in range(d["cars"].shape[1])]
    for car, row in zip(cars, d["cars"][0]):
        _set_car(car, row)
    bs = arena.ball.get_state()
    b = d["ball"][0]
    bs.pos, bs.vel, bs.ang_vel = rs.Vec(*b[0:3]), rs.Vec(*b[3:6]), rs.Vec(*b[6:9])
    arena.ball.set_state(bs)
    if "pads" in d:
        _set_pads(arena, d["pads"])

    ids = [c.id for c in cars]
    tick = [0]
    touches: list[list[int]] = [[] for _ in cars]
    goal = [-1]
    # (Touches during the step from packet t to t+1 show in packet t+1)
    arena.set_ball_touch_callback(lambda arena=None, car=None, data=None, **kw: touches[ids.index(car.id)].append(tick[0] + 1))

    def on_goal(arena=None, team=None, data=None, **kw):
        if goal[0] < 0:
            goal[0] = tick[0] + 1

    arena.set_goal_score_callback(on_goal)

    li, frame = d["last_input"], d["frame"]
    T = len(frame)
    car_s, ball_s, boost = [], [], []

    def record():
        car_s.append([[*c.get_state().pos] for c in cars])
        ball_s.append([*arena.ball.get_state().pos])
        boost.append([c.get_state().boost for c in cars])

    record()
    for t in range(T - 1):
        tick[0] = t
        for car, v in zip(cars, li[min(max(t + delay, 0), T - 1)]):
            car.set_controls(rs.CarControls(throttle=v[0], steer=v[1], pitch=v[2], yaw=v[3], roll=v[4],
                                            jump=v[5] > 0.5, boost=v[6] > 0.5, handbrake=v[7] > 0.5))
        arena.step(max(int(frame[t + 1] - frame[t]), 1))
        record()
    return {"cars": np.array(car_s), "ball": np.array(ball_s), "boost": np.array(boost), "touches": touches, "goal_tick": goal[0]}


def compare(d: dict, sim: dict) -> dict:
    T = len(d["frame"])
    car_err = np.linalg.norm(sim["cars"] - d["cars"][:, :, :3], axis=-1).max(1)
    ball_err = np.linalg.norm(sim["ball"] - d["ball"][:, :3], axis=-1)
    order = [int(x) for x in d["order"]]
    game_touch = [int(x) for x in d["touch"]]
    out = {"ticks": T}
    prev = 0
    for k, (car, gt) in enumerate(zip(order, game_touch)):
        st = [x for x in sim["touches"][car] if x > prev]
        out[f"touch{k + 1}_diff"] = (st[0] - gt) if (st and gt >= 0) else None
        prev = st[0] if st else prev
        at = gt if 0 <= gt < T else T - 1
        out[f"car_err_touch{k + 1}"] = float(car_err[at])
        out[f"ball_err_touch{k + 1}"] = float(ball_err[at])
    t1 = game_touch[0] if game_touch[0] >= 0 else T
    out["car_err_before_touch1"] = float(car_err[:t1].max())
    off = np.nonzero(car_err > 2.0)[0]
    out["car_off_2uu_tick"] = int(off[0]) if len(off) else None
    out["boost_err"] = float(np.abs(sim["boost"] - d["cars"][:, :, CAR_BOOST]).max())
    out["goal_game"] = bool(d["goal"])
    out["goal_sim"] = sim["goal_tick"] >= 0
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", type=Path)
    ap.add_argument("--delay", type=int, default=1)
    ap.add_argument("-v", "--verbose", action="store_true", help="a line per record")
    args = ap.parse_args()

    files = sorted(f for p in args.paths for f in ([p] if p.is_file() else p.glob("*.npz")))
    rows, skipped = [], []
    for f in files:
        with np.load(f, allow_pickle=True) as z:
            d = {k: z[k] for k in z.files}
        if "last_input" not in d:
            skipped.append(f.name)
            continue
        r = compare(d, replay(d, args.delay))
        r["name"] = f.name
        rows.append(r)
        if args.verbose:
            print(f"{f.name:20s} before touch1 {r['car_err_before_touch1']:8.3f}  touch1 car {r['car_err_touch1']:8.3f} ball {r['ball_err_touch1']:8.3f} "
                  f"dt {str(r['touch1_diff']):>4s}  touch2 car {r['car_err_touch2']:8.3f} ball {r['ball_err_touch2']:8.3f} dt {str(r['touch2_diff']):>4s}  "
                  f"car>2uu at {str(r['car_off_2uu_tick']):>4s}  boost {r['boost_err']:5.1f}  goal game/sim {int(r['goal_game'])}/{int(r['goal_sim'])}")
    if skipped:
        print(f"skipped {len(skipped)} records without last_input")
    if not rows:
        return

    def med(k):
        v = np.array([r[k] for r in rows])
        return f"median {np.median(v):.3f} p90 {np.percentile(v, 90):.3f}"

    def counts(k):
        v = [r[k] for r in rows if r[k] is not None]
        return dict(zip(*(x.tolist() for x in np.unique(v, return_counts=True)))) if v else {}

    print(f"{len(rows)} records (delay {args.delay})")
    print(f"car error before the first touch: {med('car_err_before_touch1')}; at it {med('car_err_touch1')}; at the second {med('car_err_touch2')}")
    print(f"ball error at the first touch: {med('ball_err_touch1')}; at the second {med('ball_err_touch2')}")
    print(f"touch tick sim - game: first {counts('touch1_diff')}, second {counts('touch2_diff')}")
    print(f"cars within 2 uu for the whole record: {sum(r['car_off_2uu_tick'] is None for r in rows)}/{len(rows)}; "
          f"boost within 1.5: {sum(r['boost_err'] <= 1.5 for r in rows)}/{len(rows)}; "
          f"goal outcome matches: {sum(r['goal_game'] == r['goal_sim'] for r in rows)}/{len(rows)}")


if __name__ == "__main__":
    main()
