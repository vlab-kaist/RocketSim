//! One arena per thread, like bench/bench_par.cpp with one arena per thread.
//!   bench_v3_par <meshes> <threads> [seconds]
use std::time::Instant;

use rocketsim::{Arena, CarBodyConfig, CarControls, GameMode, Team};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let threads: usize = args[2].parse().unwrap();
    let seconds: usize = args.get(3).map_or(15, |s| s.parse().unwrap());
    rocketsim::init(&args[1], true).unwrap();
    let t0 = Instant::now();
    let handles: Vec<_> = (0..threads)
        .map(|ti| {
            std::thread::spawn(move || {
                let mut arena = Arena::new(GameMode::Soccar);
                let cars = [arena.add_car(Team::Blue, CarBodyConfig::OCTANE), arena.add_car(Team::Orange, CarBodyConfig::OCTANE)];
                let mut h: u32 = 2654435761u32.wrapping_mul(ti as u32 + 1);
                for block in 0..seconds * 120 / 8 {
                    for &c in &cars {
                        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
                        let a = |s: u32| ((h >> s) % 3) as f32 - 1.0;
                        arena.set_car_controls(c, CarControls { throttle: a(0), steer: a(2), pitch: a(4), yaw: a(6), roll: a(8), jump: (h >> 10) % 6 == 0, boost: (h >> 13) % 3 == 0, handbrake: (h >> 15) % 8 == 0 });
                    }
                    if block % 150 == 0 {
                        arena.reset_to_random_kickoff(Some((block + ti) as u64));
                    }
                    for _ in 0..8 {
                        arena.step_tick();
                    }
                }
            })
        })
        .collect();
    for h in handles {
        h.join().unwrap();
    }
    let dt = t0.elapsed().as_secs_f64();
    let total = (seconds * 120 * threads) as f64;
    println!("threads={threads} {:.0} arena-ticks/s ({:.0} per thread)", total / dt, total / dt / threads as f64);
}
