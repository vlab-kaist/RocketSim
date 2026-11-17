//! Same workload as bench/bench.cpp in the fork, for RocketSim v3.
//!
//!   bench_v3 <meshes> [cars_per_team=1] [seconds=600] [soccar|ball]
use std::time::Instant;

use rocketsim::{Arena, CarBodyConfig, CarControls, GameMode, Team};

struct Rng(u64);
impl Rng {
    fn next_u32(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 32) as u32
    }
    fn axis(&mut self) -> f32 {
        (self.next_u32() % 3) as f32 - 1.0
    }
    fn bit(&mut self, one_in: u32) -> bool {
        self.next_u32() % one_in == 0
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let per_team: usize = args.get(2).map_or(1, |s| s.parse().unwrap());
    let seconds: usize = args.get(3).map_or(600, |s| s.parse().unwrap());
    let ball_only = args.get(4).is_some_and(|s| s == "ball");
    rocketsim::init(&args[1], true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);
    let mut cars = Vec::new();
    if !ball_only {
        for team in [Team::Blue, Team::Orange] {
            for _ in 0..per_team {
                cars.push(arena.add_car(team, CarBodyConfig::OCTANE));
            }
        }
    }
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let ticks = seconds * 120;
    let t0 = Instant::now();
    for tick in 0..ticks {
        if tick % 1200 == 0 {
            arena.reset_to_random_kickoff(Some((tick / 1200) as u64));
            if ball_only {
                let mut bs = *arena.get_ball_state();
                bs.phys.pos = rocketsim::Vec3A::new(0.0, 0.0, 800.0);
                bs.phys.vel = rocketsim::Vec3A::new((rng.next_u32() % 4000) as f32 - 2000.0, (rng.next_u32() % 4000) as f32 - 2000.0, (rng.next_u32() % 2000) as f32);
                arena.set_ball_state(bs);
            }
        }
        if tick % 8 == 0 {
            for &c in &cars {
                let cc = CarControls {
                    throttle: rng.axis(),
                    steer: rng.axis(),
                    pitch: rng.axis(),
                    yaw: rng.axis(),
                    roll: rng.axis(),
                    jump: rng.bit(6),
                    boost: rng.bit(3),
                    handbrake: rng.bit(8),
                };
                arena.set_car_controls(c, cc);
            }
        }
        arena.step_tick();
    }
    let dt = t0.elapsed().as_secs_f64();
    println!("cars={} ticks={} time={:.3}s ticks/s={:.0}", cars.len(), ticks, dt, ticks as f64 / dt);
}
