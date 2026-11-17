"""Drive the recorder with fabricated packets, without RLBot or the game."""

import numpy as np
from rlbot import flat

from rlval.record import Recorder
from rlval.recording import PHASE_RUN, Recording
from rlval.scenarios import select


class FakeInterface:
    def __init__(self):
        self.sent = []
        self.packet_handlers = []

    def send_msg(self, msg):
        self.sent.append(msg)

    def disconnect(self):
        pass


def make_packet(frame: int) -> flat.GamePacket:
    def player(team):
        return flat.PlayerInfo(
            physics=flat.Physics(location=flat.Vector3(0, frame, 17), rotation=flat.Rotator(0, 1.5, 0)),
            hitbox=flat.BoxShape(118.01, 84.2, 36.16),
            team=team,
            boost=100,
            name=f"p{team}",
            demolished_timeout=-1,
            dodge_timeout=-1,
        )

    return flat.GamePacket(
        players=[player(0), player(1)],
        balls=[flat.BallInfo(physics=flat.Physics(location=flat.Vector3(0, 0, 93)))],
        boost_pads=[flat.BoostPadState(True, 0.0)] * 34,
        match_info=flat.MatchInfo(match_phase=flat.MatchPhase.Active, frame_num=frame, seconds_elapsed=frame / 120, world_gravity_z=-650, game_speed=1),
    )


def test_recorder_runs_scenarios(tmp_path, monkeypatch):
    monkeypatch.setenv("RLBOT_AGENT_ID", "vlab/rlval-recorder")
    scenarios = [s for s in select(["input_delay_throttle", "bump_speed2300_rear"])]
    rec = Recorder(scenarios, tmp_path, repeats=1)
    rec._game_interface = FakeInterface()
    frame = 1000
    try:
        for _ in range(5000):
            # Skip a frame now and then, as a slow machine would.
            frame += 2 if frame % 97 == 0 else 1
            rec._on_packet(make_packet(frame))
    except SystemExit:
        pass
    files = sorted(tmp_path.glob("*.npz"))
    assert [f.name for f in files] == ["bump_speed2300_rear__r0.npz", "input_delay_throttle__r0.npz"]

    r = Recording.load(tmp_path / "input_delay_throttle__r0.npz")
    assert r.car_pos.shape[1:] == (2, 3) and r.last_input.shape[1:] == (2, 8)
    assert r.meta["cars"][0]["body"] == "octane" and r.meta["dropped_frames"] > 0
    run = r.phase == PHASE_RUN
    assert r.run_tick[run][0] == 0 and r.run_tick[run][-1] >= 240
    # Throttle is sent exactly for run ticks 60..179 of car 0.
    throttle = r.sent_input[run][:, 0, 0]
    ticks = r.run_tick[run]
    assert np.all(throttle[(ticks >= 60) & (ticks < 180)] == 1.0)
    assert np.all(throttle[(ticks < 60) | (ticks >= 180)] == 0.0)

    sent = rec._game_interface.sent
    assert sum(isinstance(m, flat.DesiredGameState) for m in sent) == 3  # 1 + 2 (settle == 0 sets twice)
    assert any(isinstance(m, flat.PlayerInput) and m.controller_state.throttle == 1.0 for m in sent)
