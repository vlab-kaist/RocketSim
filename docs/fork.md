# The vlab-kaist RocketSim fork

Branch `fast-step` of `vlab-kaist/RocketSim` (forked from mtheall/RocketSim `python-dev`). It is the
simulator of the rocket-league-reboot project: training (`rlrl-reboot`, pinned as a submodule), the
aerial passing planner and the visualizer use it. Changes are listed in `../CHANGELOG.md`; open and finished improvement points are in
`improvements.md` (checklist).

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
- Boost pads (v3's rule, matched to the game's pickup ticks): a car arrives on a pad when its center, taken
  half a tick back, is in the pad's cylinder (radius 176 small / 208 big, +-95 high); it stays on the pad
  while that holds or its hitbox overlaps the pad's box (96/160, +-64 around the pad). One car per pad (the
  one already on it, else the first to arrive); with several cars on one pad v3 can give it to another one.
- New car states start with `flipTime = 1` (past the 0.3 s pitch lock): the game doesn't lock pitch at
  kickoffs or after a state set.

Python binding tests (`python-mtheall/unit_test.py`, run from the directory holding `collision_meshes`; needs
`pyglm`): 84/86 pass. `TestCarConfig.test_create` checks upstream's hitbox sizes (this fork uses v3's) and
`TestArena.test_goal_event` is a scripted shot that no longer scores with this fork's physics. `test_pickle` and
`test_clone_into` (random cars) fail in about half the runs, before and after the 2026-11 changes: pickling and
`clone_into` copy through `GetState`/`SetState`, which isn't exact (positions round trip through Bullet units, last
bit), and `SetState` traces the wheels at the new pose while `GetState` reports the contacts of the trace before
the last integration, so `wheels_with_contact` can differ (the test helper then raises a TypeError on the tuples).
Exact copies are `Arena::Clone()` (Python `clone()`) and snapshots.

## Speed notes

- Current speed (2026-11-21, one core of the 7950X, `bench`): 1v1 ~820k ticks/s, 3v3 ~305k, ball only ~3.4M.
- Profile with `perf record` on `build-bench/bench` (add `-g` to the flags for source lines). The profile is
  flat; the biggest single costs are wheel ray casts, compound AABBs, orientation integration and
  quantization.
- Watch for stalled store forwarding: a vector written one component at a time (`v[i] = ...`, per-element
  constructors, arrays indexed by a computed index, chained assignments of structs) and then loaded as a
  whole stalls the load. Build vectors from scalars in registers (`btVector3(x, y, z)` does), copy structs
  before writing fresh fields into them. On Zen 4 count them with
  `perf stat -e ls_bad_status2.stli_other` (Intel: `ld_blocks.store_forward`).
- Cache footprint (`bench_par <meshes> 1 N` on one core, 8 ticks per arena visit as in training). Memory per
  1v1 arena: 290 KB (3v3 310 KB, `ArenaMemWeightMode::LIGHT` 165 KB); before the thread scratch below 540 KB.
  The biggest pieces left are the broadphase cells (~150 KB, mostly never touched), the broadphase handle
  pool (57 KB, a few handles used) and the `Arena` object itself (23 KB, mostly the dispatcher's algorithm
  table); `Car` is 2.4 KB, `Ball` 1.2 KB, `BoostPad` 128 B. Shared by all arenas: the meshes and their BVHs
  (~2.5 MB) and the broadphase static tables and static cell lists (cached by `_BuildStaticTable()` and
  `_GetStaticCells()`). What costs is not the memory an arena owns but the lines a visit touches: ~37 L2
  misses per tick in 1v1 (~300 lines, ~19 KB per 8-tick visit), spread over the cars, wheels, ball, rigid
  bodies, proxies, the `Arena` object and the cells near the bodies (no source line above 3.5% of the DRAM
  fills). Cost per tick (1v1, Ryzen 7950X, L2 1 MB, L3 32 MB per 8 cores; 2026-11-20):

  | arenas per core | L2 misses per tick | lines from DRAM per tick | cycles per tick |
  |---|---|---|---|
  | 1 | 0.4 | 0 | 6,650 |
  | 64 | 36 | 0.2 | 6,840 |
  | 256 | 37 | 1.1-1.5 | 6,960-7,120 |
  | 1024 | 37 | 16-20 | 7,800-8,000 |

  The same before the thread scratch and the idle pads: 47-48 L2 misses per tick; 7,000 / 7,190-7,340 /
  8,600-9,070 cycles at 64 / 256 / 1024 arenas. 3v3: 85 -> 73 L2 misses per tick, ~3% faster at 256-1024
  arenas. After the static meshes left the per-tick loops (2026-11-21): ~30 L2 misses per tick, 6,380 cycles at 64
  arenas (8,400-7,900 at 1024, with other jobs on the machine). The pod's Xeon 8462Y+ (L2 2 MB, L3 1.9 MB per core; before these changes), single core: 1v1
  310-330k ticks/s up to 64 arenas, 247k at 256, 240k at 1024 (-25%); 3v3 -18% at 1024 (with other users'
  jobs on the machine). Training configs run 64-128 arenas per thread; the bandwidth is low (~1 GB/s per
  core), the cost is latency.
- Thread scratch: the collision algorithms, manifolds and the solver's and island manager's buffers only live
  within a tick (the pairs are released at the end of `Step()`; contacts kept across ticks are in
  `m_rsContacts`), so the arenas stepped on one thread share that thread's
  (`btDiscreteDynamicsWorld::rsBeginThreadScratch()`); an arena's own pools are 16 entries. Idle boost pads
  (active, no cooldown, no car this tick or the last) are skipped by the per-tick pad updates; a pad's bit in
  `Arena::_boostPadsBusy` is set by `SetState()`, by a car getting on it and by `LoadSnapshot()`, so code that
  writes `BoostPad::_internalState` directly has to call `_MarkBusy()`.
- Per-tick loops go over `btCollisionWorld::rsGetDynamicObjects()` (cars and ball), not the whole collision object
  array: the ~20 static mesh bodies per arena are 850-byte objects each, and touching one costs a cache line. Their
  island state (tag -1, companion -2, hit fraction 1) is set when the list is rebuilt; nothing else changes it (the
  solver maps static bodies to its fixed body without writing them).
- Tried and dropped: skipping the rotation in the predicted transform (`predictUnconstraintMotion`, only used to
  widen the AABBs; ~2% faster and the same checksums, but which pairs overlap could change at the margin).
- Tried without effect: prefetching the next arena's cars, ball, wheels, pads and `Arena` object while the
  current one steps; transparent huge pages for the heap (`GLIBC_TUNABLES=glibc.malloc.hugetlb=1`; removes
  the TLB misses, ~3 page walks per tick, but no faster).

## Validation

`validation/` (see its README) records scenarios in the real game through RLBot v5 and replays them
open loop in this fork and in v3. Recording sets: `recordings/real-2026-11-17`, `real-2026-11-18`
(driving, jumps, ball, air touches, bounces, suspension, air bumps, flip resets), `real-2026-11-18-carcar`
(94 bump/demo scenarios x3), `real-2026-11-18-pitchlock`, `real-2026-11-18-kickoff`, `real-2026-11-19-roof`,
`real-2026-11-19-random` (55 random solo plays on the ground, in the air and on walls, x2),
`real-2026-11-19-settle` (suspension right after a state set), `real-2026-11-19-goal` (goal timing; check
with `python -m rlval.goals`).

```sh
cd validation && .venv/bin/python -m rlval.compare recordings/* --backends cpp     # all sets, ~15 s
```

Status (2026-11-21, `5f200de9`): all sets, 1076 valid recordings (48 more start with a demolished car and are
skipped), median car position error at 1 s 0.01 uu, 963 never diverge past 10 uu, demo outcomes match the
game in all of them. The scripted sets of the boost pad change (17, 18, pitchlock, carcar, roof: 879): 812
never diverge. Random solo play (`real-2026-11-19-random`, 110): median 0.05 uu at 1 s, 66 never diverge
(RocketSim v3: 4.69 uu, 39).

Recording needs the game: `RLBotServer` runs as `vlab` in the tmux session `rlbot-server`
(`/mnt/hdd0/rlbot-shared/start_server.sh`), and `python rlbot/start_match.py /mnt/hdd0/rlbot-shared/match.toml`
from any account launches Rocket League through Heroic (~30 s) in vlab's desktop session. One script at a time:
check `ss -tnp | grep 23234` for other sessions' clients (the planner's chain recorder uses the same server).

Planner chains (`python -m rlval.chain`, the planner's real-game aerial pass/shot recordings): cars
within 0.05 uu before the first touch, touch ticks as in the game.

Boost pads (`python -m rlval.pads`, pickups in the recordings): 99/137 on the same tick as the game (v3 98),
the rest mostly +-1 tick; the rule applied to the game's own recorded positions gives the same split.

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
