# Changelog (vlab-kaist fork, branch `fast-step`)

Changes of this fork against upstream `python-dev` (mtheall/RocketSim, C++ v2.2 + CPython bindings).
Newest first. Accuracy numbers are against the real-game recordings in `validation/recordings/`
(see `docs/fork.md`); "never diverging" counts recordings whose car position stays within 10 uu of the
real game for the whole run. Speed numbers are single-thread `bench/` ticks/s on the v2 server.
Bench checksums identify the exact physics; they change whenever results change.

## 2026-11-19

- **Contact points kept across ticks** (7bfbf026). One manifold per body pair is kept while the pair
  overlaps: kept points are refreshed, the tick's fresh points merged in, warm-started from the kept
  normal impulse (as the game's Bullet and RocketSim v3 do). Car-car contacts are not kept. A car's
  world contact normal is combined from all its kept world points (v3).
  Car on its roof now rests instead of rocking: roof scenarios 5-108 deg -> v3 level
  (roof_drop_h30 6.9 -> 0.02 uu). Common recordings never diverging 763 -> 775 (of 837).
  API: `ArenaSnapshot::contacts` (new field; rebuild users), Clone/LoadSnapshot carry the contacts,
  `Car/Ball::SetState` forget the body's kept contacts.
- **Air damping from the tick-start angular velocity** (f15ddee1), as the game and v3: a car pushed by
  another car's wheels no longer loses 4% spin that tick. demo_air_dz-40 55 -> 0.3 uu.
- rlval: roof/body-floor contact scenarios and recordings (e02a5479); random solo play recorded
  (`real-2026-11-19-random`, 55 x2): fork median 0.05 uu at 1 s and 66/110 never diverging, v3 4.69 uu and 39/110.

## 2026-11-18

- **Wheel extra pushback reset every trace** (8cf83ac7). A stale pushback from an earlier deep
  compression kept pushing the car; it was also hidden state that `SetState` left behind (same rows,
  different future after a goal). Never diverging 509 -> 515 (of 558); car_hits_ball off-70 55 -> 1-4 uu.
- **Cross-arena snapshots; deterministic respawns** (3a1d7222). `LoadSnapshot` accepts a snapshot from
  another arena with the same layout (car IDs translated by index; no pointers inside the snapshot);
  load only reads the snapshot (one snapshot into many arenas, many threads). Respawn locations come from
  a per-arena random state (was time-seeded, thread-local). The world inverse inertia tensor is saved.
- **No air pitch lock for new car states** (d5691895). `CarState::flipTime` defaults to 1: the game
  doesn't lock pitch at kickoffs or after a state set (measured with real kickoffs).
- rlval: recorder waits for respawns; compare skips recordings that start with a demolished car (all 45
  former "demo mismatches" were such recordings) (b47d0170); car-car re-recording (020f0cde).
- Python: `BallPredictor` uses the given ball state (a110402d).
- Determinism across threads and builds: special contact totals zeroed explicitly (g++11 left them
  uninitialized) (b6653155); valgrind uninitialized reads (8b3b17e9); data races on shared meshes (6305dcb0).
- Speed (results unchanged): car pair bounds reuse (3f95c643), tick-start states copied once
  (fda31aec), car pairs without compound algorithm objects (3c70bdbd, 668k -> 686k 1v1), pairs out of the
  hashed pair cache (6b4879ca), inline compound algorithm storage (fce75299), sorted curve arrays
  (96300fa8), wheel rays without callback objects (0dbea299), cached damping factors (9c81eb0c),
  broadphase type looked up once (07ecee00), wheel traces together and exact plane rays (367c4b44),
  cheaper traces and friction (e42712bc). 1v1 540k -> ~680k ticks/s overall.
- `Arena::SaveSnapshot()/LoadSnapshot()`: exact, allocation-free restore (475948df), Python bindings (3b9768e9).
- `Arena::Clone()` continues exactly like the original (b2a4b18d); arenas 20x cheaper to create and 100x
  cheaper to clone (c3795711).
- Multi-threaded scaling, nondeterministic car order (an `unordered_set` of pointers) and a solver bug
  (5cc5202a).
- Accuracy, porting RocketSim v3 and matching recordings: v3 hitbox sizes/offsets (d72b62d6), game wheel
  order for traces (44783fd3) and impulses (b0729439), car-car contacts to the box-box detector in pair
  order (be834670), orientation integration as the game (91fa81d0), wheels on the ball and waking a
  resting ball (1d1dfb3d), ball bounces off mesh edges such as posts and crossbar (26c3a49e), and the
  base port of v3's tick order, jumps/flips, boost, bump/demo rules and state quantization (a9fa1d64).
- Validation harness and first recordings (99a14c75).
