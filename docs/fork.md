# The vlab-kaist RocketSim fork

Branch `fast-step` of `vlab-kaist/RocketSim` (forked from mtheall/RocketSim `python-dev`). It is the
simulator of the rocket-league-reboot project: training (`rlrl-reboot`, pinned as a submodule), the
aerial passing planner and the visualizer use it. Changes are listed in `../CHANGELOG.md`.

Goals, in order: match the real game tick for tick (validated against recordings), exact determinism
(same inputs, same results, also across threads, clones and snapshots), speed.

## Building and checking

```sh
# Python module (also what validation/ uses)
cd validation && uv pip install --prerelease allow --reinstall-package RocketSim ..
# Native bench and exactness tests
cmake -S bench -B build-bench -DCMAKE_BUILD_TYPE=Release && cmake --build build-bench -j
build-bench/bench ../collision_meshes 1 3000        # 1v1 speed and checksum ("3 1000": 3v3, "1 3000 ball": ball)
build-bench/clone_test ../collision_meshes           # Arena::Clone() continues bit for bit
build-bench/snapshot_test ../collision_meshes        # LoadSnapshot(), same arena and other arenas, multi-threaded
```

The library is built with `-ffp-contract=off`, so results don't depend on `-march`. Results are exact
for the same binary and the same libm; across compilers or libm versions they may differ in the last bits.

## Determinism guarantees

- `Arena::Step()` is deterministic: no time-seeded randomness in the tick (respawn locations use a
  per-arena random state, `Arena::_respawnRandState`), no iteration over pointer-ordered containers.
- `Arena::Clone()` gives an arena that continues exactly like the original.
- `ArenaSnapshot` + `SaveSnapshot()/LoadSnapshot()`: everything the next ticks depend on (bodies,
  car/ball/pad internal state, wheel state, broadphase cell order, contact points kept across ticks,
  respawn random state). Loading into the same arena or into another arena with the same layout (same
  game mode and mutators, same cars added in the same order with the same teams and configs) continues
  bit for bit. Loading only reads the snapshot and allocates nothing in steady state, so one snapshot can
  be loaded into many arenas from many threads (the planner's search forks).
- `Car::SetState`/`Ball::SetState` leave no hidden state behind: the same states give the same future
  whatever the arena's history (they reset the wheel pushback, forget the body's kept contacts, etc.).
  `rlrl-reboot` has a regression test for this.

## Physics model notes

- Tick order, jumps/flips, boost, bump/demo rules, air control and state quantization follow RocketSim
  v3 (Rust), which was fitted to the real game; where this fork and v3 disagree, the recordings decide.
- Contact points are kept across ticks, one manifold per body pair (`btCollisionWorld::m_rsContacts`):
  kept points are refreshed, the tick's fresh points merged in (nearby point replaced, inheriting its
  normal impulse; at most 4), and the solver warm-starts from them. Car-car contacts are not kept (the
  per-tick contacts already match the game's bumps; keeping them adds impulses after cars part).
- New car states start with `flipTime = 1` (past the 0.3 s pitch lock): the game doesn't lock pitch at
  kickoffs or after a state set.

Python binding tests (`python-mtheall/unit_test.py`, run from the directory holding `collision_meshes`):
84/86 pass. `TestCarConfig.test_create` checks upstream's hitbox sizes (this fork uses v3's) and
`TestArena.test_goal_event` is a scripted shot that no longer scores with this fork's physics.

## Speed notes

- Profile with `perf record` on `build-bench/bench` (add `-g` to the flags for source lines). The profile is
  flat; the biggest single costs are wheel ray casts, compound AABBs, orientation integration and
  quantization.
- Watch for stalled store forwarding: a vector written one component at a time (`v[i] = ...`, per-element
  constructors, arrays indexed by a computed index, chained assignments of structs) and then loaded as a
  whole stalls the load. Build vectors from scalars in registers (`btVector3(x, y, z)` does), copy structs
  before writing fresh fields into them. On Zen 4 count them with
  `perf stat -e ls_bad_status2.stli_other` (Intel: `ld_blocks.store_forward`).

## Validation

`validation/` (see its README) records scenarios in the real game through RLBot v5 and replays them
open loop in this fork and in v3. Recording sets: `recordings/real-2026-11-17`, `real-2026-11-18`
(driving, jumps, ball, air touches, bounces, suspension, air bumps, flip resets), `real-2026-11-18-carcar`
(94 bump/demo scenarios x3), `real-2026-11-18-pitchlock`, `real-2026-11-18-kickoff`, `real-2026-11-19-roof`,
`real-2026-11-19-random` (55 random solo plays on the ground, in the air and on walls, x2),
`real-2026-11-19-settle` (suspension right after a state set), `real-2026-11-19-goal` (goal timing; check
with `python -m rlval.goals`).

```sh
cd validation && .venv/bin/python -m rlval.compare recordings/real-2026-11-17 recordings/real-2026-11-18 \
    recordings/real-2026-11-18-pitchlock recordings/real-2026-11-18-carcar recordings/real-2026-11-19-roof --backends cpp
```

Status (7bfbf026): 879 valid recordings (scripted scenarios), median car position error at 1 s 0.01 uu,
808 never diverge past 10 uu, demo outcomes match the game in all of them. Random solo play
(`real-2026-11-19-random`, 110): median 0.05 uu at 1 s, 66 never diverge (RocketSim v3: 4.69 uu, 39).

Planner chains (`python -m rlval.chain`, the planner's real-game aerial pass/shot recordings): cars
within 0.05 uu before the first touch, touch ticks as in the game.

Goals: `IsBallScored()` scores on the same tick as the game in all 42 goal recordings.

Known remaining differences:
- Perfectly symmetric head-on bump (`bump_headon`): which side the car tips to depends on sub-1e-5
  asymmetries; the fork tips the other way than the game (v3 matches it).
- Random play (`rand_pair_*`, `rand_ground/air/wall_*`): most divergences happen in only one of the two
  repeats of a scenario (the game's ±1 tick input timing jitter). Of the ones both repeats share, only
  `rand_ground_05` is matched by v3: a 0.15 uu/s vertical velocity difference while the car settles on its
  suspension after the state set (boosting on the ground) leaves it 0.04 uu low, so a later landing touches
  one tick early.
- Fast body landings (`roof_land_fast`, `side_land`, `nose_land`: the car body hits the floor at
  ~1000 uu/s): the fork and v3 diverge at the same tick by the same amount. On the first contact tick the
  game stops the fall almost completely (-900 -> -5 uu/s) with stronger friction, while the fork absorbs it
  over several ticks. Slow body contacts match since contacts are kept across ticks, so the game probably
  makes several contact points on the first tick (its floor may be triangles rather than a plane). Tried
  and rejected (worse overall): Bullet's plane multipoint perturbation (3 or 4 iterations, also gated on the
  kept point count as the game's persistent manifold would), the default
  split impulse threshold (-0.04), no split impulse.
