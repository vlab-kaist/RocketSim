"""Real kickoff pitch-lock probe (not a state-set scenario, so not part of rlval.record).

Start the match from match_kickoff.toml (countdown and goal reset on), then

    RLBOT_AGENT_ID=vlab/rlval-recorder python rlbot/kickoff_pitch.py out.json [rounds]

Each round scores a goal by state setting the ball, waits for the kickoff, and from 'go'
car 0 jumps for 6 ticks and then pitches; car 1 waits 60 ticks, jumps, and then pitches.
The per-tick car angular velocities show whether pitch is locked right after a kickoff.
(2026-11-18: it isn't; pitch responds from the first tick.)
"""
import json, sys
from rlbot import flat
from rlbot.managers import Script

OUT = sys.argv[1]
ROUNDS = int(sys.argv[2]) if len(sys.argv) > 2 else 3


class P(Script):
    def __init__(self):
        super().__init__("vlab/rlval-recorder")
        self.state = "score"
        self.tick = 0
        self.rows = []
        self.rounds = []
        self.sent_goal_frame = None
        self._game_interface.packet_handlers.append(self.on)

    def handle_packet(self, packet):
        pass

    def send(self, i, pitch=0.0, jump=False):
        self._game_interface.send_msg(flat.PlayerInput(i, flat.ControllerState(pitch=pitch, jump=jump)))

    def on(self, p):
        ph = p.match_info.match_phase
        f = p.match_info.frame_num
        if self.state == "score":
            if ph == flat.MatchPhase.Active and (self.sent_goal_frame is None or f - self.sent_goal_frame > 240):
                self.sent_goal_frame = f
                self._game_interface.send_msg(flat.DesiredGameState(ball_states=[flat.DesiredBallState(flat.DesiredPhysics(
                    location=flat.Vector3Partial(0, 4800, 300), velocity=flat.Vector3Partial(0, 3000, 0)))]))
            if ph in (flat.MatchPhase.GoalScored, flat.MatchPhase.Replay, flat.MatchPhase.Kickoff):
                self.state = "wait"
            return
        if self.state == "wait":
            if ph == flat.MatchPhase.Active:
                self.state, self.tick, self.rows = "run", 0, []
            else:
                return
        if self.state == "run":
            t = self.tick
            # car 0: jump at go, pitch from tick 6.  car 1: pitch on the ground from go (to see no-op), jump at tick 60, pitch from 66
            self.send(0, pitch=1.0 if 6 <= t < 60 else 0.0, jump=t < 6)
            self.send(1, pitch=1.0 if 66 <= t < 120 else 0.0, jump=60 <= t < 66)
            self.rows.append({"t": t, "phase": int(ph), "frame": f,
                              "av": [[c.physics.angular_velocity.x, c.physics.angular_velocity.y, c.physics.angular_velocity.z] for c in p.players[:2]],
                              "pos": [[c.physics.location.x, c.physics.location.y, c.physics.location.z] for c in p.players[:2]],
                              "rot": [[c.physics.rotation.pitch, c.physics.rotation.yaw, c.physics.rotation.roll] for c in p.players[:2]],
                              "in": [[c.last_input.pitch, float(c.last_input.jump)] for c in p.players[:2]]})
            self.tick += 1
            if t >= 130:
                self.rounds.append(self.rows)
                json.dump(self.rounds, open(OUT, "w"))
                self.logger.info("round %d done", len(self.rounds))
                if len(self.rounds) >= ROUNDS:
                    self._game_interface.disconnect()
                    raise SystemExit(0)
                self.state, self.sent_goal_frame = "score", None


P().run(wants_match_communications=False, wants_ball_predictions=False)
