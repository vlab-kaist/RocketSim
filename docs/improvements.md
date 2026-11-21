# Improvement checklist

Every improvement point found so far for this fork: speed, memory, accuracy, tooling. Done items say
where they landed (details in `../CHANGELOG.md` and `fork.md`); open items say what is known and the
expected gain. "Exact" means results stay bit for bit the same (bench checksums, `rlval.compare` output);
non-exact changes need the real-game validation rerun and a recorded reason.

Profile reference (1v1 `bench`, 7950X, 2026-11-21, ~820k ticks/s): wheel path ~25% (ray casts 9%, BVH ray
traversal 5.5%, vehicle update 5%, friction 2%, wheel basis/transforms 3%), solver and islands ~12%,
orientation integration 7%, tick-start quantization ~6%, collision detection (AABBs, broadphase, pairs) ~10%,
boost pads ~2.5%.

## Speed: open

- [ ] **Wheel rays: per-car cache of candidate triangles** (exact; in progress). The arena is static and a car
  moves little per tick, so the triangles near a wheel's ray can be kept and the BVH walked only when the ray
  leaves the cached box. Exact if the cache holds every triangle the ray could hit and visits them in the BVH's
  order (ties between equal hit fractions go to the first). Expected 3-8%.
- [ ] **Solver and islands specialized for a few bodies** (exact if the operation order is kept): no virtual
  calls or generic branches for constraints, multibodies, kinematic bodies; one island path. Expected a few %.
- [ ] **Tick-start quantization, the rest**: `QuantizeRotation`, quaternion normalize, `FromBasis`/`ToBasis`
  (~3%) and `QuantizeControls` in SSE, as done for `RoundVec` (exact: same IEEE operations per lane).
- [ ] **Compact per-arena layout** (exact, large): cars, wheels, ball and body state in one block per arena
  instead of Bullet's separate objects (`btRigidBody` 848 B, `btWheelInfoRL` 400 B, proxies, manifolds). The
  ~30 L2 misses per tick with many arenas per core are spread over these objects. Days of work.
- [ ] **Broadphase cells: proxy list inside the cell** (exact). A cell is a `std::vector` header plus a heap
  buffer, two lines for each cell near a body. A small inline array would make it one. A few % with many arenas.
- [ ] **Boost pads**: `_CheckCollide` and the grid lookup ~2.5%; skip cars that can't reach any pad this tick.
- [ ] **Pod measurement of the current build** (Xeon 8462Y+, single core, 64-1024 arenas): the pod numbers in
  `fork.md` predate the thread scratch.
- [ ] **Consumers' compile flags**: `btGameMath` uses `std::fma`; without `-mfma` (e.g. a default x86-64 build of
  the Python module or rlrl) each one is a libm call. Check what rlrl and the wheel builds use.

## Speed: decided against, or tried without effect

- [x] ~~Predicted transform without the rotation~~ (`predictUnconstraintMotion`, only widens the AABBs): ~2%
  faster, same checksums, but which pairs overlap can change at the margin. Not exact; dropped 2026-11-21.
- [x] ~~Contact generation as RocketSim v3 (box-triangle SAT, sphere-triangle)~~: the game runs Bullet and this
  fork matches the recordings much better than v3 (random play 0.05 vs 4.69 uu at 1 s); porting v3's detectors
  would likely cost accuracy.
- [x] ~~Prefetching the next arena's objects~~: no effect (2026-11-20).
- [x] ~~Transparent huge pages for the heap~~: removes the TLB misses, no faster (2026-11-20).
- [x] ~~Orientation integration (7%)~~: a serial chain of sqrt, divide and the game's sin/cos; no exact
  shortcut found.

## Speed: done

- [x] Static meshes out of the per-tick loops, quantization `RoundVec` in SSE (2026-11-21, 843c27af): +5% 1v1.
- [x] Thread scratch for collision pools and solver buffers, idle boost pads skipped (2026-11-20, 5f200de9):
  memory per arena 540 -> 290 KB, +10% with 1024 arenas per core.
- [x] No stalled store forwarding (2026-11-19, 2825dbd7): 611k -> 717k 1v1.
- [x] 2026-11-18 series (see CHANGELOG): car pair bounds reuse, tick-start states copied once, car pairs without
  compound algorithms, pairs out of the hashed pair cache, inline compound algorithm storage, sorted curves, wheel
  rays without callback objects, cached damping, broadphase type looked up once, wheel traces together and exact
  plane rays, cheaper traces and friction, shared broadphase static tables; arenas 20x cheaper to create and 100x
  cheaper to clone; multi-threaded scaling.

## Memory (RSS only; little speed expected)

- [ ] Broadphase handle pool, 57 KB per arena, a few handles used.
- [ ] `Arena` object 23 KB, mostly the dispatcher's algorithm table.
- [ ] Broadphase cells ~150 KB per arena, mostly never touched (they hold dynamic proxy membership across ticks,
  so they can't be shared).
- [x] Collision pools and solver buffers per thread (2026-11-20).

## Accuracy: open (from `fork.md`, Known remaining differences)

- [ ] Fast body landings (`roof_land_fast`, `side_land`, `nose_land`): the game stops a ~1000 uu/s body landing on
  the first contact tick; the fork (and v3) over several. Probably several contact points on the first tick in the
  game (triangle floor?). Tried and rejected: plane multipoint perturbation, other split impulse settings.
- [ ] Perfectly symmetric head-on bump (`bump_headon`): tips the other way than the game (v3 matches).
- [ ] `rand_ground_05`: 0.15 uu/s vertical velocity difference while settling after a state set while boosting.
- [ ] Boost pads: 99/137 pickups on the game's tick, the rest mostly +-1 (the rule applied to the game's own
  positions gives the same split, so likely the game's own timing).

## Tooling and tests

- [ ] Python `test_pickle`/`test_clone_into` fail in about half the runs by design (`SetState` retraces the
  wheels; see `fork.md`). Could make the tests compare against a state round trip instead.
- [x] Real-game recording through the v2 RLBotServer works (checked 2026-11-21; how-to in `fork.md`).
- [x] Validation over all sets in ~15 s (`rlval.compare recordings/*`).
