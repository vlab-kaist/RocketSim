"""RLBot v5 script that runs the scenarios in the real game and records them.

Run it against a match started from `rlbot/match.toml` (see README). The script
needs state setting enabled; with that enabled the server accepts player input
for any car from this script, so no separate bot processes are required.

    RLBOT_AGENT_ID=vlab/rlval-recorder python -m rlval.record --out recordings/ [patterns...]
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from rlbot import flat
from rlbot.managers import Script
from rlbot.version import __version__ as RLBOT_VERSION

from .common import TICK_RATE, body_from_hitbox
from .recording import PHASE_RUN, PHASE_SETTLE, PHASE_SETUP, RecordingBuilder
from .scenarios import Body, Scenario, select

AGENT_ID = "vlab/rlval-recorder"
SETUP_TICKS = 12  # packets to wait after a state set before trusting the state


def _vec(v) -> list[float]:
    return [v.x, v.y, v.z]


def _rot(r) -> list[float]:
    return [r.pitch, r.yaw, r.roll]


def _controls_list(c: flat.ControllerState) -> list[float]:
    return [c.throttle, c.steer, c.pitch, c.yaw, c.roll, float(c.jump), float(c.boost), float(c.handbrake)]


def _controller(vals: list[float]) -> flat.ControllerState:
    return flat.ControllerState(
        throttle=vals[0],
        steer=vals[1],
        pitch=vals[2],
        yaw=vals[3],
        roll=vals[4],
        jump=vals[5] > 0.5,
        boost=vals[6] > 0.5,
        handbrake=vals[7] > 0.5,
    )


def _desired_physics(b: Body) -> flat.DesiredPhysics:
    return flat.DesiredPhysics(
        location=flat.Vector3Partial(*b.pos),
        rotation=flat.RotatorPartial(pitch=b.pitch, yaw=b.yaw, roll=b.roll),
        velocity=flat.Vector3Partial(*b.vel),
        angular_velocity=flat.Vector3Partial(*b.ang_vel),
    )


class Recorder(Script):
    def __init__(self, scenarios: list[Scenario], out_dir: Path, repeats: int):
        super().__init__(AGENT_ID)
        self.queue = [(s, r) for s in scenarios for r in range(repeats)]
        self.out_dir = out_dir
        self.current: Scenario | None = None
        self.repeat = 0
        self.builder: RecordingBuilder | None = None
        self.phase = PHASE_SETUP
        self.phase_start_frame = 0
        self.run_start_frame = 0
        self.prev_frame: int | None = None
        self.dropped = 0
        self.start_score = 0
        # Every packet goes through this handler, not only the newest one, so the
        # recording has no gaps even if this process falls behind.
        self._game_interface.packet_handlers.append(self._on_packet)

    def initialize(self):
        self.logger.info("recorder ready, %d scenario runs queued", len(self.queue))

    def handle_packet(self, packet: flat.GamePacket) -> None:
        pass  # all work happens in _on_packet

    # -- scenario control -------------------------------------------------

    def _send_inputs(self, controls: list[list[float]]) -> None:
        for i, vals in enumerate(controls):
            self._game_interface.send_msg(flat.PlayerInput(i, _controller(vals)))

    def _set_state(self, s: Scenario) -> None:
        # Script.set_game_state is broken in rlbot 2.0.0b56 (wrong argument count),
        # so the message is built and sent directly.
        state = flat.DesiredGameState(
            ball_states=[flat.DesiredBallState(_desired_physics(s.ball))],
            car_states=[flat.DesiredCarState(_desired_physics(c), c.boost) for c in s.cars],
        )
        self._game_interface.send_msg(state)

    def _begin(self, packet: flat.GamePacket) -> bool:
        if not self.queue:
            return False
        self.current, self.repeat = self.queue.pop(0)
        s = self.current
        if len(packet.players) < len(s.cars):
            raise RuntimeError(f"scenario {s.name} needs {len(s.cars)} cars, match has {len(packet.players)}")
        meta = {
            "scenario": s.name,
            "repeat": self.repeat,
            "tags": s.tags,
            "source": "rlbot",
            "rlbot_interface": RLBOT_VERSION,
            "game_mode": str(self.match_config.game_mode),
            "map": self.match_config.game_map_upk,
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "gravity_z": packet.match_info.world_gravity_z,
            "game_speed": packet.match_info.game_speed,
            "cars": [
                {
                    "name": p.name,
                    "team": p.team,
                    "hitbox": [p.hitbox.length, p.hitbox.width, p.hitbox.height],
                    "hitbox_offset": _vec(p.hitbox_offset),
                    "body": body_from_hitbox(p.hitbox.length, p.hitbox.width, p.hitbox.height),
                }
                for p in packet.players[: len(s.cars)]
            ],
            "pads": [{"pos": _vec(p.location), "big": p.is_full_boost} for p in self.field_info.boost_pads],
            "settle": s.settle,
            "duration": s.duration,
        }
        self.builder = RecordingBuilder(meta, len(s.cars))
        self.phase = PHASE_SETUP
        self.phase_start_frame = packet.match_info.frame_num
        self.dropped = 0
        self._set_state(s)
        return True

    def _finish(self, ended_by: str = "duration") -> None:
        assert self.builder is not None and self.current is not None
        self.builder.meta["dropped_frames"] = self.dropped
        self.builder.meta["ended_by"] = ended_by
        rec = self.builder.build()
        path = self.out_dir / f"{self.current.name}__r{self.repeat}.npz"
        rec.save(path)
        self.logger.info("saved %s (%d packets, %d dropped frames, ended by %s)", path.name, len(rec.frame_num), self.dropped, ended_by)
        self.current = None
        self.builder = None

    # -- per packet -------------------------------------------------------

    def _row(self, packet: flat.GamePacket, run_tick: int) -> dict:
        n = self.builder.num_cars
        players = packet.players[:n]
        ball = packet.balls[0].physics
        return {
            "frame_num": packet.match_info.frame_num,
            "seconds": packet.match_info.seconds_elapsed,
            "phase": self.phase,
            "run_tick": run_tick,
            "ball_pos": _vec(ball.location),
            "ball_rot": _rot(ball.rotation),
            "ball_vel": _vec(ball.velocity),
            "ball_ang_vel": _vec(ball.angular_velocity),
            "car_pos": [_vec(p.physics.location) for p in players],
            "car_rot": [_rot(p.physics.rotation) for p in players],
            "car_vel": [_vec(p.physics.velocity) for p in players],
            "car_ang_vel": [_vec(p.physics.angular_velocity) for p in players],
            "car_boost": [p.boost for p in players],
            "car_air_state": [int(p.air_state) for p in players],
            "car_supersonic": [p.is_supersonic for p in players],
            "car_has_jumped": [p.has_jumped for p in players],
            "car_has_double_jumped": [p.has_double_jumped for p in players],
            "car_has_dodged": [p.has_dodged for p in players],
            "car_dodge_timeout": [p.dodge_timeout for p in players],
            "car_dodge_elapsed": [p.dodge_elapsed for p in players],
            "car_dodge_dir": [[p.dodge_dir.x, p.dodge_dir.y] for p in players],
            "car_demolished_timeout": [p.demolished_timeout for p in players],
            "car_touch_seconds": [p.latest_touch.game_seconds if p.latest_touch else -1.0 for p in players],
            "last_input": [_controls_list(p.last_input) for p in players],
            "pad_active": [p.is_active for p in packet.boost_pads],
            "pad_timer": [p.timer for p in packet.boost_pads],
        }

    def _score(self, packet: flat.GamePacket) -> int:
        return sum(t.score for t in packet.teams)

    def _on_packet(self, packet: flat.GamePacket) -> None:
        if packet.match_info.match_phase != flat.MatchPhase.Active or not packet.balls:
            self.prev_frame = None
            if self.current is not None and self.phase == PHASE_RUN:
                self._finish(ended_by="match_phase")
            return
        if self.current is not None and self.phase == PHASE_RUN and self._score(packet) != self.start_score:
            # A goal teleports the ball; keep what was recorded before it, and the run tick of the first
            # packet that shows the goal
            self.builder.meta["goal_run_tick"] = packet.match_info.frame_num - self.run_start_frame
            self._finish(ended_by="goal")
            return
        frame = packet.match_info.frame_num
        if self.prev_frame is not None and frame - self.prev_frame > 1:
            self.dropped += frame - self.prev_frame - 1
        self.prev_frame = frame

        if self.current is None and any(p.demolished_timeout >= 0 for p in packet.players):
            # A car demolished at the end of the previous run is still waiting to respawn; a state set
            # doesn't bring it back, so the next scenario would run without it
            return
        if self.current is None and not self._begin(packet):
            self.logger.info("all scenarios recorded")
            self._game_interface.disconnect()
            raise SystemExit(0)
        s = self.current
        elapsed = frame - self.phase_start_frame

        if self.phase == PHASE_SETUP and elapsed >= SETUP_TICKS:
            self.phase, self.phase_start_frame, elapsed = PHASE_SETTLE, frame, 0
            if s.settle == 0:
                # Velocity scenarios must not coast before the run, so the state is
                # set again in the same packet the run starts. The first run packets
                # still show the old state; the replay starts after that teleport.
                self._set_state(s)
        if self.phase == PHASE_SETTLE and elapsed >= s.settle:
            self.phase, self.phase_start_frame, elapsed = PHASE_RUN, frame, 0
            self.run_start_frame = frame
            self.start_score = self._score(packet)

        run_tick = frame - self.run_start_frame if self.phase == PHASE_RUN else -1
        index = self.builder.add(self._row(packet, run_tick))
        controls = s.controls_at(run_tick) if self.phase == PHASE_RUN else s.controls_at(-1)
        self._send_inputs(controls)
        self.builder.set_sent(index, controls)

        if self.phase == PHASE_RUN and run_tick >= s.duration:
            self._finish()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("patterns", nargs="*", help="substrings of scenario names or tags; default is all")
    ap.add_argument("--out", type=Path, default=Path("recordings"))
    ap.add_argument("--repeats", type=int, default=1, help="record each scenario this many times")
    ap.add_argument("--list", action="store_true", help="print the selected scenarios and exit")
    args = ap.parse_args()

    scenarios = select(args.patterns)
    if args.list:
        for s in scenarios:
            print(f"{s.name:40s} {s.duration / TICK_RATE:5.1f}s  {','.join(s.tags)}")
        print(f"{len(scenarios)} scenarios")
        return
    Recorder(scenarios, args.out, args.repeats).run(wants_match_communications=False, wants_ball_predictions=False)


if __name__ == "__main__":
    main()
