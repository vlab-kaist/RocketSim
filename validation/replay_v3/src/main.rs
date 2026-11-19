//! Replay backend for RocketSim v3 (Rust).
//!
//! Reads a replay job as JSON on stdin and writes the result as JSON on stdout.
//! The job and result formats are described in `rlval/job.py`.
//! Usage: replay_v3 <collision_meshes_dir>

use std::io::{Read, Write};

use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, BallState, BoostPadState, CarBodyConfig, CarControls, CarCore, GameMode, Mat3A, MutatorConfig,
    PhysState, Team, Vec3A,
};
use serde::{Deserialize, Serialize};

#[derive(Deserialize)]
struct CarSpec {
    team: u8,
    body: String,
    pos: [f32; 3],
    rot: [[f32; 3]; 3],
    vel: [f32; 3],
    ang_vel: [f32; 3],
    boost: f32,
    on_ground: bool,
    has_jumped: bool,
    has_double_jumped: bool,
    has_flipped: bool,
    demoed: bool,
    supersonic: bool,
    #[serde(default)]
    handbrake: f32,
}

#[derive(Deserialize)]
struct BallSpec {
    pos: [f32; 3],
    rot: [[f32; 3]; 3],
    vel: [f32; 3],
    ang_vel: [f32; 3],
}

#[derive(Deserialize)]
struct Step {
    ticks: u32,
    controls: Vec<[f32; 8]>,
}

#[derive(Deserialize)]
struct Job {
    game_mode: String,
    gravity_z: f32,
    cars: Vec<CarSpec>,
    ball: BallSpec,
    steps: Vec<Step>,
    /// Boost pad cooldowns at the start (seconds, 0 = active), in RLBot pad order
    #[serde(default)]
    pads: Option<Vec<f32>>,
}

#[derive(Serialize)]
struct Event {
    step: usize,
    attacker: usize,
    victim: usize,
    demo: bool,
}

#[derive(Serialize)]
struct Output {
    backend: String,
    ball: Vec<Vec<f32>>,
    cars: Vec<Vec<Vec<f32>>>,
    events: Vec<Event>,
}

fn phys(pos: [f32; 3], rot: [[f32; 3]; 3], vel: [f32; 3], ang_vel: [f32; 3]) -> PhysState {
    PhysState {
        pos: Vec3A::from(pos),
        rot_mat: Mat3A::from_cols(Vec3A::from(rot[0]), Vec3A::from(rot[1]), Vec3A::from(rot[2])),
        vel: Vec3A::from(vel),
        ang_vel: Vec3A::from(ang_vel),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let meshes = std::env::args().nth(1).ok_or("usage: replay_v3 <collision_meshes_dir>")?;
    let mut input = String::new();
    std::io::stdin().read_to_string(&mut input)?;
    let job: Job = serde_json::from_str(&input)?;

    rocketsim::init(&meshes, true)?;

    let game_mode = match job.game_mode.as_str() {
        "soccar" => GameMode::Soccar,
        "hoops" => GameMode::Hoops,
        "heatseeker" => GameMode::Heatseeker,
        "snowday" => GameMode::Snowday,
        "dropshot" => GameMode::Dropshot,
        other => return Err(format!("unknown game mode {other}").into()),
    };
    let mut mutators = MutatorConfig::new(game_mode);
    mutators.gravity = Vec3A::new(0.0, 0.0, job.gravity_z);
    let mut arena = Arena::new_with_config(ArenaConfig::new(game_mode).with_mutators(mutators).with_rng_seed(0));

    for spec in &job.cars {
        let body = match spec.body.as_str() {
            "octane" => CarBodyConfig::OCTANE,
            "dominus" => CarBodyConfig::DOMINUS,
            "plank" => CarBodyConfig::PLANK,
            "breakout" => CarBodyConfig::BREAKOUT,
            "hybrid" => CarBodyConfig::HYBRID,
            "merc" => CarBodyConfig::MERC,
            other => return Err(format!("unknown car body {other}").into()),
        };
        let team = if spec.team == 0 { Team::Blue } else { Team::Orange };
        let idx = arena.add_car(team, body);
        let core = CarCore {
            phys: phys(spec.pos, spec.rot, spec.vel, spec.ang_vel),
            boost: spec.boost,
            has_jumped: spec.has_jumped,
            has_double_jumped: spec.has_double_jumped,
            has_flipped: spec.has_flipped,
            ..CarCore::DEFAULT
        };
        arena.set_car_state(idx, core);
        arena.edit_car_state_full(idx, |state| {
            state.is_supersonic = spec.supersonic;
            state.handbrake_val = spec.handbrake;
            state.set_contact_flags(spec.on_ground, [spec.on_ground; 4]);
        });
        if spec.demoed {
            arena.demolish_car(idx);
        }
    }

    arena.set_ball_state(BallState {
        phys: phys(job.ball.pos, job.ball.rot, job.ball.vel, job.ball.ang_vel),
        ..BallState::DEFAULT
    });

    if let Some(pads) = &job.pads {
        for (idx, &cooldown) in pads.iter().enumerate().take(arena.num_boost_pads()) {
            arena.set_boost_pad_state(idx, BoostPadState { cooldown, touching_car_idxs: Vec::new() });
        }
    }

    let num_cars = job.cars.len();
    let mut out = Output { backend: "v3".to_string(), ball: Vec::new(), cars: Vec::new(), events: Vec::new() };

    for (step_idx, step) in job.steps.iter().enumerate() {
        for (idx, vals) in step.controls.iter().enumerate().take(num_cars) {
            arena.set_car_controls(idx, CarControls::from_floats(*vals, 0.5));
        }
        for _ in 0..step.ticks {
            for event in arena.step_tick() {
                if let ArenaEvent::CarHitCar(hit) = event {
                    out.events.push(Event {
                        step: step_idx,
                        attacker: hit.bumper_car_idx,
                        victim: hit.victim_car_idx,
                        demo: hit.is_demo,
                    });
                }
            }
        }

        let ball = arena.get_ball_state().phys;
        let mut row = Vec::with_capacity(9);
        row.extend(ball.pos.to_array());
        row.extend(ball.vel.to_array());
        row.extend(ball.ang_vel.to_array());
        out.ball.push(row);

        let mut cars = Vec::with_capacity(num_cars);
        for idx in 0..num_cars {
            let state = arena.get_car_state(idx);
            let mut row = Vec::with_capacity(19);
            row.extend(state.phys.pos.to_array());
            row.extend(state.phys.get_forward_dir().to_array());
            row.extend(state.phys.get_up_dir().to_array());
            row.extend(state.phys.vel.to_array());
            row.extend(state.phys.ang_vel.to_array());
            row.push(state.boost);
            row.push(f32::from(state.is_on_ground()));
            row.push(f32::from(state.is_supersonic));
            row.push(f32::from(state.is_demoed));
            cars.push(row);
        }
        out.cars.push(cars);
    }

    let stdout = std::io::stdout();
    let mut lock = stdout.lock();
    serde_json::to_writer(&mut lock, &out)?;
    lock.flush()?;
    Ok(())
}
