"""Scripted validation scenarios.

A scenario places the ball and cars with state setting, waits for them to settle,
then plays a fixed control script. Only state that RLBot can set is used
(physics and boost), so every scenario starts from a grounded, neutral car unless
it sets velocities directly.

Coordinates are Unreal units. Car 0 is blue, car 1 is orange.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .common import CONTROL_NAMES, NUM_CONTROLS

CAR_REST_Z = 17.01
BALL_REST_Z = 93.15
FAR_AWAY = (3000.0, -4000.0)  # where unused cars and the ball are parked


@dataclass
class Body:
    pos: tuple[float, float, float]
    yaw: float = 0.0  # radians
    vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    ang_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pitch: float = 0.0
    roll: float = 0.0
    boost: float = 100.0


@dataclass
class Segment:
    """Controls held for ticks [start, end) of the run phase."""

    start: int
    end: int
    controls: dict[str, float]

    def __post_init__(self):
        unknown = set(self.controls) - set(CONTROL_NAMES)
        if unknown:
            raise ValueError(f"unknown controls: {unknown}")


@dataclass
class Scenario:
    name: str
    cars: list[Body]
    ball: Body
    scripts: list[list[Segment]]  # one list of segments per car
    duration: int  # run-phase ticks
    settle: int = 90  # ticks of neutral input between state set and run
    tags: list[str] = field(default_factory=list)

    def controls_at(self, tick: int) -> list[list[float]]:
        """Control values for every car at run-phase tick `tick`."""
        out = []
        for script in self.scripts:
            vals = [0.0] * NUM_CONTROLS
            for seg in script:
                if seg.start <= tick < seg.end:
                    for k, v in seg.controls.items():
                        vals[CONTROL_NAMES.index(k)] = float(v)
            out.append(vals)
        return out


def _parked_ball() -> Body:
    return Body(pos=(FAR_AWAY[0], FAR_AWAY[1], BALL_REST_Z))


def _parked_car(i: int = 0) -> Body:
    return Body(pos=(-FAR_AWAY[0], FAR_AWAY[1] + 300.0 * i, CAR_REST_Z), yaw=math.pi / 2)


def _solo(name, car, script, duration, ball=None, settle=90, tags=()):
    return Scenario(
        name=name,
        cars=[car, _parked_car()],
        ball=ball or _parked_ball(),
        scripts=[script, []],
        duration=duration,
        settle=settle,
        tags=list(tags),
    )


def _origin_car(yaw=math.pi / 2, y=-4000.0, x=0.0, **kw) -> Body:
    return Body(pos=(x, y, CAR_REST_Z), yaw=yaw, **kw)


def driving() -> list[Scenario]:
    out = []
    out.append(_solo("input_delay_throttle", _origin_car(), [Segment(60, 180, {"throttle": 1})], 240, tags=["delay"]))
    out.append(_solo("input_delay_jump", _origin_car(), [Segment(60, 70, {"jump": 1})], 240, tags=["delay"]))
    out.append(_solo("throttle_straight", _origin_car(), [Segment(0, 480, {"throttle": 1})], 600, tags=["drive"]))
    out.append(_solo("boost_straight", _origin_car(), [Segment(0, 300, {"throttle": 1, "boost": 1})], 480, tags=["drive", "boost"]))
    out.append(_solo("reverse", _origin_car(y=3000.0), [Segment(0, 360, {"throttle": -1})], 480, tags=["drive"]))
    out.append(
        _solo(
            "throttle_then_brake",
            _origin_car(),
            [Segment(0, 240, {"throttle": 1, "boost": 1}), Segment(240, 420, {"throttle": -1})],
            480,
            tags=["drive"],
        )
    )
    out.append(_solo("coast", _origin_car(), [Segment(0, 180, {"throttle": 1, "boost": 1})], 720, tags=["drive"]))
    for steer in (0.25, 0.5, 1.0):
        for boost in (0, 1):
            out.append(
                _solo(
                    f"turn_steer{steer}_boost{boost}",
                    _origin_car(y=-2000.0),
                    [Segment(0, 120, {"throttle": 1, "boost": boost}), Segment(120, 600, {"throttle": 1, "boost": boost, "steer": steer})],
                    600,
                    tags=["drive", "steer"],
                )
            )
    out.append(
        _solo(
            "powerslide",
            _origin_car(y=-2000.0),
            [Segment(0, 180, {"throttle": 1, "boost": 1}), Segment(180, 300, {"throttle": 1, "steer": 1, "handbrake": 1}), Segment(300, 480, {"throttle": 1})],
            540,
            tags=["drive", "handbrake"],
        )
    )
    return out


def jumping() -> list[Scenario]:
    out = []
    for hold in (1, 6, 12, 24, 40):
        out.append(_solo(f"jump_hold{hold}", _origin_car(), [Segment(30, 30 + hold, {"jump": 1})], 300, tags=["jump"]))
    out.append(
        _solo("double_jump", _origin_car(), [Segment(30, 42, {"jump": 1}), Segment(54, 60, {"jump": 1})], 360, tags=["jump"])
    )
    flips = {"front": {"pitch": -1}, "back": {"pitch": 1}, "left": {"yaw": -1}, "right": {"yaw": 1}, "diag": {"pitch": -1, "yaw": 1}}
    for name, stick in flips.items():
        for moving in (0, 1):
            pre = [Segment(0, 150, {"throttle": 1, "boost": 1})] if moving else []
            t0 = 150 if moving else 30
            out.append(
                _solo(
                    f"flip_{name}_moving{moving}",
                    _origin_car(),
                    pre + [Segment(t0, t0 + 6, {"jump": 1}), Segment(t0 + 12, t0 + 18, {"jump": 1, **stick})],
                    t0 + 330,
                    tags=["jump", "flip"],
                )
            )
    out.append(
        _solo(
            "flip_cancel",
            _origin_car(),
            [Segment(0, 150, {"throttle": 1, "boost": 1}), Segment(150, 156, {"jump": 1}), Segment(162, 168, {"jump": 1, "pitch": -1}), Segment(168, 240, {"pitch": 1})],
            480,
            tags=["jump", "flip"],
        )
    )
    for axis in ("pitch", "yaw", "roll"):
        out.append(
            _solo(
                f"air_{axis}",
                _origin_car(),
                [Segment(30, 54, {"jump": 1}), Segment(60, 120, {axis: 1}), Segment(120, 150, {axis: -1})],
                420,
                tags=["air"],
            )
        )
    out.append(
        _solo(
            "aerial_boost",
            _origin_car(),
            [Segment(30, 54, {"jump": 1, "pitch": 1}), Segment(54, 90, {"pitch": 1, "boost": 1}), Segment(90, 240, {"boost": 1})],
            720,
            tags=["air", "boost"],
        )
    )
    return out


def ball() -> list[Scenario]:
    out = []
    for z, vel in ((600.0, (0.0, 0.0, 0.0)), (300.0, (800.0, 600.0, 400.0)), (1000.0, (2500.0, 100.0, -500.0))):
        out.append(
            Scenario(
                name=f"ball_drop_z{int(z)}_v{int(vel[0])}",
                cars=[_parked_car(0), _parked_car(1)],
                ball=Body(pos=(0.0, 0.0, z), vel=vel),
                scripts=[[], []],
                duration=900,
                settle=0,
                tags=["ball"],
            )
        )
    out.append(
        Scenario(
            name="ball_spin_bounce",
            cars=[_parked_car(0), _parked_car(1)],
            ball=Body(pos=(0.0, 0.0, 500.0), vel=(500.0, 0.0, 0.0), ang_vel=(0.0, 5.0, 3.0)),
            scripts=[[], []],
            duration=900,
            settle=0,
            tags=["ball"],
        )
    )
    for boost, offset in ((0, 0.0), (1, 0.0), (1, 40.0), (1, -70.0)):
        out.append(
            _solo(
                f"car_hits_ball_boost{boost}_off{int(offset)}",
                _origin_car(y=-2500.0),
                [Segment(0, 600, {"throttle": 1, "boost": boost})],
                600,
                ball=Body(pos=(offset, 0.0, BALL_REST_Z)),
                tags=["ball", "car_ball"],
            )
        )
    out.append(
        _solo(
            "flip_into_ball",
            _origin_car(y=-2200.0),
            [Segment(0, 600, {"throttle": 1}), Segment(110, 116, {"jump": 1}), Segment(122, 128, {"jump": 1, "pitch": -1})],
            600,
            ball=Body(pos=(0.0, 0.0, BALL_REST_Z)),
            tags=["ball", "car_ball", "flip"],
        )
    )
    return out


AIR_Z = 500.0  # height for airborne collision tests
APPROACH_DIST = 600.0  # attacker start distance; short so the set speed is still held at impact


def _approach(name, speed, victim: Body, attacker_yaw_offset=0.0, lateral=0.0, vel_yaw_offset=0.0, hold=True, z=CAR_REST_Z, tags=()):
    """Attacker starts APPROACH_DIST behind the origin travelling at `speed` along +Y.

    `attacker_yaw_offset` rotates the attacker body away from its velocity (a slide,
    or driving in reverse at pi). `vel_yaw_offset` rotates the velocity direction away
    from the line to the victim. `lateral` shifts the attacker sideways so the hit is
    off-centre.

    The speed comes from state setting. Throttle without boost holds it: above the
    throttle top speed (1410) throttle neither accelerates nor brakes, and below it
    the short approach adds little. `hold=False` coasts instead.
    """
    travel = math.pi / 2 + vel_yaw_offset
    vel = (speed * math.cos(travel), speed * math.sin(travel), 0.0)
    attacker = Body(pos=(lateral, -APPROACH_DIST, z), yaw=travel + attacker_yaw_offset, vel=vel)
    reversing = abs(attacker_yaw_offset) > math.pi / 2
    airborne = z > CAR_REST_Z + 1.0
    script = [Segment(0, 240, {"throttle": -1.0 if reversing else 1.0})] if hold and not airborne else []
    return Scenario(
        name=name,
        cars=[attacker, victim],
        ball=_parked_ball(),
        scripts=[script, []],
        duration=300,
        settle=0,
        tags=list(tags),
    )


def bumps_and_demos() -> list[Scenario]:
    out = []
    victim_yaws = {"rear": math.pi / 2, "front": -math.pi / 2, "side": 0.0}
    # Speed sweep across the bump range and both supersonic thresholds (2100 maintain, 2200 start).
    for speed in (600, 1000, 1400, 1800, 2050, 2150, 2190, 2210, 2300):
        for vname, vyaw in victim_yaws.items():
            out.append(
                _approach(
                    f"bump_speed{speed}_{vname}",
                    speed,
                    Body(pos=(0.0, 0.0, CAR_REST_Z), yaw=vyaw),
                    tags=["car_car", "demo" if speed >= 2200 else "bump"],
                )
            )
    # Off-centre hits at supersonic: where does the demo cone end?
    for lateral in (20, 40, 60, 80, 100):
        out.append(
            _approach(f"demo_lateral{lateral}", 2300, Body(pos=(0.0, 0.0, CAR_REST_Z), yaw=math.pi / 2), lateral=float(lateral), tags=["car_car", "demo", "cone"])
        )
    # Cone tests are done in the air: wheel friction would straighten a sliding car
    # before impact, and two cars released at the same height fall together.
    # Attacker body yawed away from its velocity. Demo yaw limit is expected near 45 degrees.
    for deg in (15, 30, 40, 44, 47, 50, 60, 75, 90, 135, 165, 180):
        out.append(
            _approach(
                f"demo_air_yaw{deg}",
                2300,
                Body(pos=(0.0, 0.0, AIR_Z), yaw=0.0),
                attacker_yaw_offset=math.radians(deg),
                z=AIR_Z,
                tags=["car_car", "demo", "cone", "air"] + (["reverse"] if deg > 90 else []),
            )
        )
    # Victim above or below the attacker. Demo pitch limit is expected near 37 degrees.
    for dz in (-60, -40, -20, 20, 40, 60, 80):
        out.append(
            _approach(f"demo_air_dz{dz}", 2300, Body(pos=(0.0, 0.0, AIR_Z + dz), yaw=0.0), z=AIR_Z, tags=["car_car", "demo", "cone", "air"])
        )
    # Velocity not pointing at the victim: the attacker passes with a glancing hit.
    for deg in (2, 4, 6, 8):
        out.append(
            _approach(f"demo_air_glance{deg}", 2300, Body(pos=(0.0, 0.0, AIR_Z), yaw=0.0), vel_yaw_offset=math.radians(deg), z=AIR_Z, tags=["car_car", "demo", "cone", "air"])
        )
    # Reverse at speed on the ground (only reachable by state setting).
    out.append(_approach("demo_reverse", 2300, Body(pos=(0.0, 0.0, CAR_REST_Z), yaw=0.0), attacker_yaw_offset=math.pi, tags=["car_car", "demo", "reverse"]))
    # State cannot be set mid-run without disturbing the attacker, so the victim is
    # dropped from a height that puts it `dz` above rest when the attacker arrives.
    # It is falling at about 135 uu/s at impact.
    fall_time = (APPROACH_DIST - 118.0) / 2300.0
    drop = 0.5 * 650.0 * fall_time**2
    for dz in (20, 40, 60, 80, 120):
        victim = Body(pos=(0.0, 0.0, CAR_REST_Z + dz + drop), yaw=0.0)
        out.append(_approach(f"demo_victim_up{dz}", 2300, victim, tags=["car_car", "demo", "cone", "air"]))
    # Two moving cars: head-on and chase.
    for vspeed, label in ((-1500.0, "headon"), (1200.0, "chase")):
        victim = Body(pos=(0.0, 300.0, CAR_REST_Z), yaw=math.pi / 2, vel=(0.0, vspeed, 0.0))
        out.append(_approach(f"demo_{label}", 2300, victim, tags=["car_car", "demo"]))
        out.append(_approach(f"bump_{label}", 1600, victim, tags=["car_car", "bump"]))
    # Supersonic grace: coast from 2300 (about 525 uu/s^2 of drag) so that speed at
    # impact is above 2200, between 2100 and 2200, or below 2100.
    for dist in (300, 550, 800, 1100):
        victim = Body(pos=(0.0, float(dist) - APPROACH_DIST, CAR_REST_Z), yaw=0.0)
        out.append(_approach(f"demo_coast_dist{dist}", 2300, victim, hold=False, tags=["car_car", "demo", "supersonic_grace"]))
    return out


# Octane geometry used to aim contacts (uu, relative to the car origin)
OCTANE_NOSE_X = 13.88 + 120.51 / 2  # front face of the hitbox
OCTANE_CENTER_Z = 20.75  # hitbox centre height
OCTANE_TOP_Z = 20.75 + 38.66 / 2
BALL_RADIUS = 91.25


def _air_pair(name, car: Body, ball: Body, script=(), duration=300, tags=()):
    return Scenario(name=name, cars=[car, _parked_car()], ball=ball, scripts=[list(script), []], duration=duration, settle=0, tags=list(tags))


def air_touches() -> list[Scenario]:
    """Car and ball both airborne, so only the car-ball contact differs from free flight.

    Both fall together under gravity, so the relative motion is a straight line. The
    contact happens about 0.2 s into the run and the ball is then followed to the floor.
    """
    out = []
    z = 900.0
    gap_time = 0.2
    # Nose hits: approach speed, sideways and vertical offset of the ball from the nose line
    for speed in (600.0, 1400.0, 2200.0):
        for lat, dz in ((0.0, 0.0), (45.0, 0.0), (-80.0, 0.0), (0.0, 50.0), (0.0, -45.0), (60.0, 60.0)):
            car = Body(pos=(0.0, -(OCTANE_NOSE_X + BALL_RADIUS + speed * gap_time), z), yaw=math.pi / 2, vel=(0.0, speed, 0.0))
            ball = Body(pos=(lat, 0.0, z + OCTANE_CENTER_Z + dz))
            out.append(_air_pair(f"airtouch_nose_v{int(speed)}_lat{int(lat)}_dz{int(dz)}", car, ball, tags=["ball", "car_ball", "air_touch"]))
    # Ball already moving towards the car, and ball with spin
    for bvel, spin, label in (((0.0, -1500.0, 200.0), (0.0, 0.0, 0.0), "incoming"), ((0.0, -800.0, 0.0), (4.0, 0.0, 3.0), "spin"), ((600.0, -600.0, 400.0), (0.0, -5.0, 0.0), "diag")):
        car = Body(pos=(0.0, -600.0, z), yaw=math.pi / 2, vel=(0.0, 900.0, 0.0))
        ball = Body(pos=(0.0, 0.0, z + OCTANE_CENTER_Z), vel=bvel, ang_vel=spin)
        out.append(_air_pair(f"airtouch_ball_{label}", car, ball, tags=["ball", "car_ball", "air_touch"]))
    # Pitched and rolled car bodies (the contact lands on the bumper edge, roof or underside)
    for pitch, roll, label in ((0.6, 0.0, "noseup"), (-0.6, 0.0, "nosedown"), (0.0, 0.8, "rolled"), (1.4, 0.0, "vertical")):
        car = Body(pos=(0.0, -(OCTANE_NOSE_X + BALL_RADIUS + 1400.0 * gap_time), z), yaw=math.pi / 2, pitch=pitch, roll=roll, vel=(0.0, 1400.0, 0.0))
        ball = Body(pos=(0.0, 0.0, z + OCTANE_CENTER_Z))
        out.append(_air_pair(f"airtouch_body_{label}", car, ball, tags=["ball", "car_ball", "air_touch"]))
    # Roof and underside hits: car moving up into the ball, or down onto it
    for vz, label in ((1200.0, "roof"), (-1200.0, "underside")):
        sign = 1.0 if vz > 0 else -1.0
        car = Body(pos=(0.0, 0.0, z - sign * (OCTANE_TOP_Z + BALL_RADIUS + abs(vz) * gap_time)), yaw=math.pi / 2, vel=(0.0, 0.0, vz))
        ball = Body(pos=(20.0, 0.0, z))
        out.append(_air_pair(f"airtouch_{label}", car, ball, tags=["ball", "car_ball", "air_touch"]))
    # Spinning car hits the ball (angular velocity changes the contact velocity)
    for angv, label in (((0.0, 0.0, 5.5), "yawspin"), ((5.5, 0.0, 0.0), "pitchspin"), ((0.0, 5.5, 0.0), "rollspin")):
        car = Body(pos=(0.0, -(OCTANE_NOSE_X + BALL_RADIUS + 1000.0 * gap_time), z), yaw=math.pi / 2, vel=(0.0, 1000.0, 0.0), ang_vel=angv)
        ball = Body(pos=(0.0, 0.0, z + OCTANE_CENTER_Z))
        out.append(_air_pair(f"airtouch_{label}", car, ball, tags=["ball", "car_ball", "air_touch"]))
    # Car flipping into a floating ball (dodge torque during the touch)
    car = Body(pos=(0.0, -500.0, z), yaw=math.pi / 2, vel=(0.0, 1000.0, 0.0))
    ball = Body(pos=(0.0, 0.0, z + OCTANE_CENTER_Z))
    out.append(_air_pair("airtouch_dodge", car, ball, script=[Segment(5, 10, {"jump": 1, "pitch": -1})], tags=["ball", "car_ball", "air_touch", "flip"]))
    return out


def ball_bounces() -> list[Scenario]:
    """Ball against walls, corners, ceiling, goal posts and the crossbar, with and without spin."""
    out = []
    shots = {
        "sidewall": ((3000.0, 0.0, 600.0), (2500.0, 300.0, 200.0)),
        "backwall": ((0.0, 4000.0, 1000.0), (300.0, 2500.0, 200.0)),
        "corner": ((3000.0, 4000.0, 300.0), (1500.0, 1500.0, 300.0)),
        "ceiling": ((0.0, 0.0, 1500.0), (400.0, 300.0, 1600.0)),
        "wall_floor_seam": ((3300.0, -1000.0, 200.0), (2500.0, 0.0, -600.0)),
        "post": ((893.0, 3800.0, 300.0), (0.0, 2500.0, 0.0)),
        "crossbar": ((0.0, 3800.0, 680.0), (0.0, 2500.0, 0.0)),
        "into_goal": ((0.0, 3500.0, 300.0), (0.0, 2500.0, 100.0)),
        "side_ramp": ((2800.0, 2000.0, 100.0), (2000.0, 0.0, 0.0)),
    }
    for label, (pos, vel) in shots.items():
        for spin, slabel in (((0.0, 0.0, 0.0), ""), ((3.0, -4.0, 2.0), "_spin")):
            out.append(
                Scenario(
                    name=f"bounce_{label}{slabel}",
                    cars=[_parked_car(0), _parked_car(1)],
                    ball=Body(pos=pos, vel=vel, ang_vel=spin),
                    scripts=[[], []],
                    duration=480,
                    settle=0,
                    tags=["ball", "bounce"],
                )
            )
    # Ball rolling with spin on the floor (rolling friction and spin decay)
    out.append(
        Scenario(
            name="ball_roll_spin",
            cars=[_parked_car(0), _parked_car(1)],
            ball=Body(pos=(0.0, -2000.0, BALL_REST_Z), vel=(0.0, 1200.0, 0.0), ang_vel=(0.0, 0.0, 6.0)),
            scripts=[[], []],
            duration=480,
            settle=0,
            tags=["ball", "bounce"],
        )
    )
    return out


def suspension() -> list[Scenario]:
    """Wheel contacts: tilted landings, partial contact, landing on the roof, wall driving."""
    out = []
    # Drops from a height with the body tilted, so the wheels touch one by one
    for pitch, roll, label in ((0.0, 0.0, "flat"), (0.3, 0.0, "nose_up"), (-0.3, 0.0, "nose_down"), (0.0, 0.35, "roll"), (0.25, -0.3, "pitch_roll"), (0.0, 1.2, "side"), (0.0, math.pi, "roof")):
        out.append(_solo(f"land_{label}", Body(pos=(0.0, -3000.0, 250.0), yaw=math.pi / 2, pitch=pitch, roll=roll), [], 300, settle=0, tags=["suspension", "land"]))
    # Landing while moving and steering (lateral friction at touch-down)
    for vel, label in (((0.0, 1200.0, -300.0), "moving"), ((800.0, 1000.0, -500.0), "sideways")):
        out.append(_solo(f"land_{label}", Body(pos=(0.0, -3000.0, 200.0), yaw=math.pi / 2, roll=0.2, vel=vel), [Segment(0, 300, {"throttle": 1, "steer": 0.5})], 300, settle=0, tags=["suspension", "land"]))
    # Driving onto the side wall and up the curved transition, then along the wall
    out.append(_solo("wall_drive", Body(pos=(3200.0, -2000.0, CAR_REST_Z), yaw=0.0, vel=(1200.0, 0.0, 0.0)), [Segment(0, 360, {"throttle": 1, "boost": 1}), Segment(120, 360, {"steer": 0.4})], 420, settle=0, tags=["suspension", "wall"]))
    out.append(_solo("wall_drive_down", Body(pos=(3200.0, -2000.0, CAR_REST_Z), yaw=0.0, vel=(1400.0, 0.0, 0.0)), [Segment(0, 150, {"throttle": 1, "boost": 1}), Segment(150, 420, {"throttle": 1, "steer": 1})], 480, settle=0, tags=["suspension", "wall"]))
    # Corner curve and the goal-line ramp
    out.append(_solo("corner_drive", Body(pos=(2500.0, 3800.0, CAR_REST_Z), yaw=0.6, vel=(1000.0, 800.0, 0.0)), [Segment(0, 360, {"throttle": 1})], 420, settle=0, tags=["suspension", "wall"]))
    # Driving over a small bump: the car runs over the ball at low speed
    out.append(_solo("drive_onto_ball", _origin_car(y=-800.0), [Segment(0, 300, {"throttle": 0.6})], 360, ball=Body(pos=(0.0, 0.0, BALL_REST_Z)), tags=["suspension", "car_ball"]))
    # Dribble: ball dropped onto the roof of a slowly driving car
    out.append(_solo("dribble_drop", _origin_car(y=-1500.0, vel=(0.0, 300.0, 0.0)), [Segment(0, 360, {"throttle": 0.3})], 360, ball=Body(pos=(0.0, -1440.0, 260.0), vel=(0.0, 300.0, 0.0)), settle=0, tags=["suspension", "car_ball"]))
    # Handbrake turn and hard braking: large suspension load transfer
    out.append(_solo("hard_brake", _origin_car(y=-3000.0, vel=(0.0, 2000.0, 0.0)), [Segment(0, 240, {"throttle": -1})], 300, settle=0, tags=["suspension"]))
    out.append(_solo("handbrake_spin", _origin_car(y=-3000.0, vel=(0.0, 1600.0, 0.0)), [Segment(0, 200, {"throttle": 1, "steer": 1, "handbrake": 1})], 300, settle=0, tags=["suspension", "handbrake"]))
    return out


def air_bumps() -> list[Scenario]:
    """Car-car contacts in the air below demo speed: contact response, not bump rules."""
    out = []
    z = 800.0
    for speed in (500.0, 1200.0, 1900.0):
        for vyaw, label in ((0.0, "side"), (math.pi / 2, "rear"), (math.pi / 4, "angled")):
            attacker = Body(pos=(0.0, -(250.0 + speed * 0.2), z), yaw=math.pi / 2, vel=(0.0, speed, 0.0))
            victim = Body(pos=(0.0, 0.0, z), yaw=vyaw)
            out.append(Scenario(name=f"airbump_v{int(speed)}_{label}", cars=[attacker, victim], ball=_parked_ball(), scripts=[[], []], duration=300, settle=0, tags=["car_car", "air_bump"]))
    # Both cars moving and rotating
    attacker = Body(pos=(0.0, -500.0, z), yaw=math.pi / 2, vel=(0.0, 1000.0, 0.0), ang_vel=(0.0, 0.0, 3.0))
    victim = Body(pos=(30.0, 0.0, z + 20.0), yaw=0.3, vel=(0.0, -400.0, 0.0), ang_vel=(2.0, 0.0, 0.0))
    out.append(Scenario(name="airbump_spinning", cars=[attacker, victim], ball=_parked_ball(), scripts=[[], []], duration=300, settle=0, tags=["car_car", "air_bump"]))
    return out


def flip_resets() -> list[Scenario]:
    """Wheels touching the ball in the air give the flip back.

    The car spends its jump first (a double jump), then touches the ball, then tries
    to dodge. The dodge only happens if the touch reset the flip, so the trajectory
    after the dodge input shows whether the game and the simulator agree.
    """
    out = []
    z = 700.0
    # Upside-down car rising into a ball above it: the wheels face the ball
    variants = {
        "full": dict(roll=math.pi, lat=0.0, vz=900.0),
        "tilted": dict(roll=math.pi - 0.35, lat=0.0, vz=900.0),
        "pitched": dict(roll=math.pi, pitch=0.3, lat=0.0, vz=900.0),
        "offset": dict(roll=math.pi, lat=55.0, vz=900.0),
        "fast": dict(roll=math.pi, lat=0.0, vz=1500.0),
        "roof": dict(roll=0.0, lat=0.0, vz=900.0),  # roof touch: no reset expected
    }
    for label, v in variants.items():
        car = Body(pos=(0.0, 0.0, z), yaw=math.pi / 2, roll=v["roll"], pitch=v.get("pitch", 0.0), vel=(0.0, 0.0, v["vz"]))
        ball = Body(pos=(v["lat"], 0.0, z + 200.0 + v["vz"] * 0.05))
        script = [Segment(3, 8, {"jump": 1}), Segment(70, 76, {"jump": 1, "pitch": -1})]
        out.append(_air_pair(f"flipreset_{label}", car, ball, script=script, duration=300, tags=["ball", "car_ball", "flip_reset"]))
    # Reset then a second double jump instead of a dodge
    car = Body(pos=(0.0, 0.0, z), yaw=math.pi / 2, roll=math.pi, vel=(0.0, 0.0, 900.0))
    ball = Body(pos=(0.0, 0.0, z + 245.0))
    out.append(_air_pair("flipreset_doublejump", car, ball, script=[Segment(3, 8, {"jump": 1}), Segment(70, 76, {"jump": 1})], duration=300, tags=["ball", "car_ball", "flip_reset"]))
    return out


def _random_script(rng, duration: int, air: bool) -> list[Segment]:
    """Random analog controls, changed every 6 to 30 ticks, with occasional jumps, boost and handbrake."""
    out, t = [], 0
    while t < duration:
        end = min(duration, t + rng.randint(6, 30))
        controls = {
            "throttle": rng.choice([1.0, 1.0, 0.0, -1.0, rng.uniform(-1, 1)]),
            "steer": rng.choice([0.0, rng.uniform(-1, 1), rng.uniform(-1, 1)]),
            "pitch": rng.uniform(-1, 1) if air or rng.random() < 0.3 else 0.0,
            "yaw": rng.uniform(-1, 1) if air or rng.random() < 0.3 else 0.0,
            "roll": rng.uniform(-1, 1) if rng.random() < (0.6 if air else 0.2) else 0.0,
            "boost": 1.0 if rng.random() < 0.35 else 0.0,
            "handbrake": 1.0 if rng.random() < 0.15 else 0.0,
        }
        out.append(Segment(t, end, controls))
        if rng.random() < 0.25:
            # A short jump press (a jump, a second jump or a flip, depending on the state)
            j = rng.randint(t, end - 1)
            out.append(Segment(j, min(end, j + rng.randint(1, 8)), {"jump": 1.0}))
        t = end
    return out


def random_play(count_ground: int = 25, count_air: int = 20, count_pair: int = 15, count_wall: int = 10, seed: int = 2026_11_18) -> list[Scenario]:
    """Seeded random states and controls, so that no scenario is symmetric (no ties for the simulators to break)."""
    import random

    rng = random.Random(seed)
    out = []
    duration = 360

    def rand_yaw():
        return rng.uniform(-math.pi, math.pi)

    for i in range(count_ground):
        yaw = rand_yaw()
        speed = rng.uniform(0, 2000)
        x, y = rng.uniform(-3000, 3000), rng.uniform(-4000, 4000)
        car = Body(pos=(x, y, CAR_REST_Z), yaw=yaw, vel=(speed * math.cos(yaw), speed * math.sin(yaw), 0.0), boost=rng.uniform(20, 100))
        d, a = rng.uniform(300, 1200), rng.uniform(-0.6, 0.6)
        bx, by = x + d * math.cos(yaw + a), y + d * math.sin(yaw + a)
        bvel = (rng.uniform(-600, 600), rng.uniform(-600, 600), rng.choice([0.0, rng.uniform(0, 600)]))
        ball = Body(pos=(max(-3500, min(3500, bx)), max(-4500, min(4500, by)), BALL_REST_Z + rng.choice([0.0, rng.uniform(0, 300)])), vel=bvel,
                    ang_vel=(rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3, 3)))
        out.append(Scenario(f"rand_ground_{i:02d}", [car, _parked_car(1)], ball, [_random_script(rng, duration, air=False), []], duration, settle=0, tags=["random", "random_ground"]))

    for i in range(count_air):
        x, y, z = rng.uniform(-2500, 2500), rng.uniform(-3500, 3500), rng.uniform(300, 1500)
        car = Body(pos=(x, y, z), yaw=rand_yaw(), pitch=rng.uniform(-1.4, 1.4), roll=rng.uniform(-math.pi, math.pi),
                   vel=(rng.uniform(-1000, 1000), rng.uniform(-1000, 1000), rng.uniform(-500, 800)),
                   ang_vel=(rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-4, 4)), boost=rng.uniform(20, 100))
        d = rng.uniform(250, 700)
        dx, dy, dz = rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 0.5)
        n = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
        ball = Body(pos=(x + d * dx / n, y + d * dy / n, max(200.0, min(1800.0, z + d * dz / n))),
                    vel=(rng.uniform(-800, 800), rng.uniform(-800, 800), rng.uniform(-300, 500)),
                    ang_vel=(rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-4, 4)))
        out.append(Scenario(f"rand_air_{i:02d}", [car, _parked_car(1)], ball, [_random_script(rng, duration, air=True), []], duration, settle=0, tags=["random", "random_air"]))

    for i in range(count_pair):
        x, y = rng.uniform(-2500, 2500), rng.uniform(-3500, 3500)
        yaw = rand_yaw()
        gap = rng.uniform(500, 1500)
        sa, sb = rng.uniform(800, 2200), rng.uniform(0, 1800)
        yb = yaw + math.pi + rng.uniform(-1.2, 1.2)
        ox, oy = rng.uniform(-150, 150), rng.uniform(-150, 150)
        car_a = Body(pos=(x, y, CAR_REST_Z), yaw=yaw, vel=(sa * math.cos(yaw), sa * math.sin(yaw), 0.0), boost=rng.uniform(20, 100))
        car_b = Body(pos=(x + gap * math.cos(yaw) + ox, y + gap * math.sin(yaw) + oy, CAR_REST_Z), yaw=yb, vel=(sb * math.cos(yb), sb * math.sin(yb), 0.0), boost=rng.uniform(20, 100))
        ball = Body(pos=(x + 0.5 * gap * math.cos(yaw) + rng.uniform(-400, 400), y + 0.5 * gap * math.sin(yaw) + rng.uniform(-400, 400), BALL_REST_Z),
                    vel=(rng.uniform(-400, 400), rng.uniform(-400, 400), 0.0))
        out.append(Scenario(f"rand_pair_{i:02d}", [car_a, car_b], ball, [_random_script(rng, duration, air=False), _random_script(rng, duration, air=False)], duration, settle=0,
                            tags=["random", "random_pair", "car_car"]))

    for i in range(count_wall):
        side = rng.choice([-1, 1])
        y = rng.uniform(-3000, 3000)
        yaw = (0.0 if side > 0 else math.pi) + rng.uniform(-0.7, 0.7)
        speed = rng.uniform(600, 2000)
        car = Body(pos=(side * rng.uniform(2800, 3500), y, CAR_REST_Z), yaw=yaw, vel=(speed * math.cos(yaw), speed * math.sin(yaw), 0.0), boost=rng.uniform(20, 100))
        ball = Body(pos=(side * 3600.0, y + rng.uniform(-800, 800), rng.uniform(BALL_REST_Z, 900)), vel=(rng.uniform(-300, 300), rng.uniform(-600, 600), rng.uniform(-200, 400)))
        out.append(Scenario(f"rand_wall_{i:02d}", [car, _parked_car(1)], ball, [_random_script(rng, duration, air=False), []], duration, settle=0, tags=["random", "random_wall"]))
    return out


def pitch_lock() -> list[Scenario]:
    """Whether air pitch is locked right after a state set.

    The simulator ignores air pitch while less than 0.3 s have passed since the last flip, and a state set (or a
    kickoff reset) starts that time at 0. These check whether the game does the same: pitch from the first tick
    in the air, after waiting in the air, and right after a jump from the ground; yaw as a control.
    """
    out = []
    for axis in ("pitch", "yaw"):
        out.append(_solo(f"pitchlock_air_now_{axis}", Body(pos=(0.0, -3000.0, 1200.0), yaw=math.pi / 2), [Segment(0, 72, {axis: 1})], 120, settle=0, tags=["air", "pitch_lock"]))
    out.append(_solo("pitchlock_air_wait", Body(pos=(0.0, -3000.0, 1600.0), yaw=math.pi / 2), [Segment(60, 120, {"pitch": 1})], 150, settle=0, tags=["air", "pitch_lock"]))
    out.append(_solo("pitchlock_jump_now", _origin_car(), [Segment(0, 6, {"jump": 1}), Segment(6, 60, {"pitch": 1})], 120, settle=0, tags=["jump", "pitch_lock"]))
    return out


def all_scenarios() -> list[Scenario]:
    scenarios = driving() + jumping() + ball() + bumps_and_demos() + air_touches() + ball_bounces() + suspension() + air_bumps() + flip_resets() + random_play() + pitch_lock()
    names = [s.name for s in scenarios]
    assert len(names) == len(set(names)), "duplicate scenario names"
    return scenarios


def select(patterns: list[str] | None) -> list[Scenario]:
    """Scenarios whose name or tags contain any of the given substrings."""
    scenarios = all_scenarios()
    if not patterns:
        return scenarios
    return [s for s in scenarios if any(p in s.name or p in s.tags for p in patterns)]
