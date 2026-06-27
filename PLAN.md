# HW2 Plan

> Revised 2026-06-10 against the **9.6.26 skeleton update** and the updated
> Assignment 2 DRAFT PDF. Submission deadline: June-21st, 23:30.

## Context
HW2 is a refactoring exercise that takes our HW1 drone-mapping simulator and
migrates it onto the course-supplied `ex_2_skeleton-main/` skeleton, whose
interfaces are locked (must be used verbatim) and whose I/O formats change
(`.npy` maps + YAML configs instead of HW1's `map_input.txt` and key:value
configs). It also requires GTest/GMock component + integration tests, a
hierarchical YAML output report, and immediate error logging.

We are not writing fresh code where HW1 already has working logic. Every
meaningful behaviour comes from the corresponding HW1 file, adapted to the new
types and I/O.

Working tree lives in `cpp/hw2/`: skeleton contents are copied here, then HW1
logic is refactored into the copied files. HW1 stays untouched.

## 9.6.26 skeleton update — what changed (already synced into hw2/)
- `IMap3D`: `get()` -> `atVoxel()`, `resolution()` -> `getMapConfig()`
  returning new `types::MapConfig{ MappingBounds boundaries, Position3D
  offset, PhysicalLength resolution }`.
- `MappingBounds` moved from MissionTypes to MapTypes (plain aggregate now;
  all config structs lost their deleted-default-ctor boilerplate).
- `MissionConfigData` **lost `boundaries`** (bounds now live on
  MapConfig/IMap3D); `output_mapping_resolution_factor` is now `double`.
- `MissionRunResult` lost `score` + `output_map_file` (moved to
  `SimulationResult`); single `ErrorRef` -> `std::vector<ErrorRef> errors`.
- `SimulationConfigData` gained `Position3D map_offset` (YAML
  `map_axes_offset`: NPY cell (0,0,0) sits at this world position).
- `ISimulationRun::run()` now returns `types::SimulationResult` (carries
  sim+mission configs, `resolution_request_status`, `mission_results`,
  `output_map_file`, `output_map_config`, `mission_score`). Score computation
  moved **out of MissionControl into SimulationRunImpl** (which now also takes
  sim config, mission config, and output map path in its ctor).
- `ISimulation::run()` returns `types::SimulationManagerReport{
  generated_at_utc, metric, tuple<double,double> score_range, error_score,
  vector<SimulationResult> runs }` — flat run list, no more score groups.
- `Map3DImpl` is now **dense**: holds `shared_ptr<NpyArray>` (TinyNPY) +
  `MapConfig`; ctors `(map_ptr)` / `(map_ptr, MapConfig)`. NPY loading moved
  to the factory (`loadNpyArray` helper in SimulationRunFactoryImpl.cpp).
- `LidarHit` **lost its `bool hit` field** — a miss is encoded as
  max-double-cm distance only. Skeleton MockLidar already updated (and its
  ray-march is now fairly complete; step = `0.1 * resolution`).
- `MapsComparison::compare` is now
  `static std::vector<double> compare(const IMap3D& origin, const
  std::vector<IMap3D*> targets)` (>= 1 target). `ResolutionRatio` is gone.
- `maps_comparison` CLI: `./maps_comparison <origin> <target>
  [comparison_config=<path>]` — optional YAML (per-map `map_res_cm`,
  `map_offset`, `map_boundaries`); no config => same offset/bounds/resolution.
  **Different-resolution comparison is now an optional bonus** (we skip it).
- Skeleton vcpkg.json now ships yaml-cpp (>=0.9.0) + version-pinned mp-units;
  `cpp_yaml_example/` added with an `example_yml` target as a yaml-cpp
  reference.

## Decisions (confirmed with user; re-validated against the update)
- YAML: yaml-cpp (now course-provided in skeleton vcpkg.json).
- Algo split: BFS port lives in `MappingAlgorithmImpl`; `DroneControlImpl`
  only orchestrates per-step scan/apply/move.
- Scans/step: DroneControl calls `lidar.scan()` 6 times per step (HW1 dirs:
  +/-X, +/-Y, +/-Z).
- Bonuses: none (explicitly includes the new different-resolution comparison
  bonus — mismatched resolutions in maps_comparison are an error path).
- Tests: `hw2/tests/components/` + `hw2/tests/integration/`, single
  `drone_mapper_simulation_test` exe (GTest+GMock). Suite names must match the
  PDF filters exactly: SimulationManager, SimulationRun, MissionControl,
  DroneControl, MappingAlgorithm, MockLidar, MapsComparison, Integration.
- Resolution policy: always use input resolution; every run reports
  `resolution_request_status = Ignored` (now a per-`SimulationResult` field,
  not per group). YAML `output_mapping_resolution_factor`: integer, default 1
  if missing, `< 1` -> error log + Ignored (struct field is `double`).
- Output layout (unchanged):
  `<out>/output_results/<sim_stem>/<mission_stem>/<drone_stem>__<lidar_stem>/{output_map.npy,error_log.txt}`
  plus `<out>/simulation_output.yaml`; documented in `readme.txt`.
- MockLidar: keep skeleton ctor + field names; verify/align its (now nearly
  complete) ray-march against HW1 semantics instead of wholesale replacement.
- Reuse: HW1 logic refactored in-place into HW2 files.
- `cpp_yaml_example/` + `example_yml` target kept in hw2 for now as yaml-cpp
  reference; optionally drop before submission.

## Mission boundaries gap (new; design decision)
The mission YAML still has `boundaries`, but `MissionConfigData` no longer
carries them and `ISimulationRunFactory::create(simulation, mission, drone,
lidar, output_path)` is locked. Chosen mechanism: the YAML loader returns a
`LoadedComposition { SimulationCompositionData data; std::vector<MappingBounds>
mission_bounds; }` (parallel to `data.missions`). `main` hands the factory a
pointer to the composition + the bounds vector before calling
`SimulationManager::run`. Inside `create()`, the mission index is recovered via
pointer arithmetic (`&mission - &composition->missions[0]`, valid because
SimulationManager iterates the same vectors by const ref). The factory then
builds the output `MapConfig{ bounds, offset = bounds min corner, gps_res }`.

## Interfaces - DO NOT EDIT
`I*` headers under `include/drone_mapper/` and all `types::` structs are
locked (now including `MapConfig`/`MappingBounds`). `Map3DImpl.h` public ctors
are course-prescribed — keep the header verbatim (helpers live in the .cpp
anonymous namespace). Other Impl ctors (`*Impl`, `Mock*`) may take additional
dependencies where wiring needs more; update `SimulationRunFactoryImpl.cpp`
accordingly.

## Execution order (one file/group per session)

1. **Bootstrap** — DONE (and redone after the 9.6.26 update): skeleton synced
   into `hw2/`, skeleton vcpkg.json adopted (yaml-cpp included), CMakeLists =
   new skeleton base + yaml-cpp linked PRIVATE to drone_mapper +
   `enable_testing()` + `add_subdirectory(tests)`. tests/CMakeLists.txt globs
   components/ + integration/. Build verification still pending (run in the
   course devcontainer: configure with vcpkg toolchain, `cmake --build`).

2. **`Map3DImpl`** — DONE (rewritten for the new dense design):
   - Storage = the `shared_ptr<NpyArray>` itself (no sparse hashmap).
     World->index: `i = floor((pos - offset)/res)` per axis; cell (0,0,0) at
     `offset`; C-order flat index `(ix*ny + iy)*nz + iz`.
   - Loaded maps (hidden / comparison inputs): ctor validates 3-D shape +
     dtype (`u`/`b` uint8, `i` int8); 1 -> Occupied, 0 -> Empty, -1 ->
     Unmapped (uint8 reinterpreted as int8 so 255 reads back Unmapped).
   - Empty array + non-empty bounds => allocates dense int8 grid covering
     `[offset, bounds max)`, filled Unmapped (output-map mode). Requires
     `offset <= bounds min` and `res > 0`.
   - `atVoxel`: outside boundaries (or array footprint when no boundaries
     configured) -> OutOfBounds; inside bounds but past array extent -> Empty
     (hidden-map semantics); else raw value.
   - `set`: no-op outside bounds/array; writes through dtype-aware path.
   - `save`: `SaveNPY` of the dense array as-is.
   - NOTE: skeleton's factory stub passes empty bounds + empty NpyArray for
     the output map — that yields an unusable map by design; step 12 must pass
     real mission bounds. VoxelKey.h kept (used later by MappingAlgorithmImpl
     internals); ParseUtils.h kept (trim/tryDouble only — nothing obsolete).

3. **`MockGPS`** — verify skeleton's; ensure `heading()` returns Orientation
   with altitude=0 (HW1 drone has no pitch). Add `setPosition` / `setHeading`
   used by MockMovement.

4. **`MockMovement`** — port HW1 `MockDroneMovementDriver`. Extend ctor to
   `(MockGPS&, const IMap3D& world, DroneConfigData)` — resolution comes from
   `world.getMapConfig().resolution`, no separate param.
   `rotate/advance/elevate` return `MovementResult{success,msg}` (no throws).
   Body cross-section sweep against the world map identical to HW1, using
   `atVoxel`. `rotate(direction, angle)`: signed angle, clamp to +/-max_rotate,
   update GPS heading. `advance/elevate`: clamp, body-sweep along path; on
   collision return `{false, "collision"}`; otherwise update GPS position.

5. **`MockLidar`** — verify/align the skeleton's updated ray-march with HW1
   `MockLidarSensor` semantics: ring radius spacing (HW1 used
   `circle_spacing/2`), miss = max-double distance (NO `hit` flag anymore),
   closer-than-z_min hit returns 0 distance, step granularity (skeleton:
   `0.1 * getMapConfig().resolution`). Keep ctor `(LidarConfigData, const
   IMap3D&, const IGPS&)` and the `z_min`/`z_max`/`d`/`fov_circles` names.

6. **`ScanResultToVoxels`** — port HW1 `Drone::processBeam`. Signature
   unchanged. Miss detection is now distance-based: hit iff
   `distance <= z_max` (max-double => miss); `distance == 0` means
   closer-than-z_min — mark nothing along that beam. For each hit: abs beam =
   heading + hit.angle; march sub-voxel to hit (intermediate voxels Empty, hit
   voxel Occupied) or to z_max (full path Empty). Step ~1 cm fallback (no
   resolution param on the static signature).

7. **`MappingAlgorithmImpl`** — port HW1 `Drone.cpp` BFS into here.
   - Extend ctor: `(MissionConfigData, DroneConfigData, MapConfig
     output_map_config)` — bounds no longer come from MissionConfigData.
   - State: internal sparse map (`VoxelKey` hashmap) at gps resolution within
     the MapConfig bounds, frontier queue, visited + in_frontier sets.
   - `applyVoxelUpdates`: write each voxel to the internal map.
   - `nextMove`: seed visited/frontier on first call; expand frontier from
     current voxel using internal map + drone-fit check (HW1 `canFitAt`); pop
     next reachable target via BFS pathfind; translate next-step delta into
     one `MovementCommand` (Rotate -> Elevate -> Advance, Hover on done).

8. **`DroneControlImpl::step()`** (ctor already takes output_map + algo refs):
   1. snapshot state from gps: `{gps.position(), gps.heading(), step_index_}`.
   2. for each of 6 HW1 dirs: `scan = lidar.scan(dir)`,
      `voxels = ScanResultToVoxels::convert(state.position, state.heading,
      scan)`, then `mapping_algo.applyVoxelUpdates(voxels)` and
      `output_map.set(v.position, v.value)` per voxel.
   3. `cmd = mapping_algo.nextMove(state, last_scan)`.
   4. translate cmd -> `movement.rotate/advance/elevate`. fail ->
      `DroneStepResult{Error, msg}`.
   5. Hover from algo -> `DroneStepResult{Completed, "frontier exhausted"}`.
      Else `step_index_++; Continue`.

9. **`MissionControlImpl::runMission`** — NO scoring here anymore:
   - loop `mission_.max_steps` times calling `drone_control_.step()`.
   - Error -> log immediately, return `{Error, step,
     {ErrorRef{"DRONE_STEP_FAILED", msg}}}` (errors is a vector now).
   - Completed -> break with status Completed; loop exhausted -> MaxSteps.
   - `output_map_.save(output_map_file_)` (save failure -> Error + ErrorRef).

10. **`SimulationRunImpl::run()`** — real work now (was a one-liner):
    - `mission_result = mission_control_->runMission()`.
    - score: status Error -> -1.0; else
      `MapsComparison::compare(*hidden_map_, {output_map_.get()})[0]`.
    - return `SimulationResult{ simulation_config_, mission_config_,
      ResolutionRequestStatus::Ignored, {mission_result}, output_map_file_,
      output_map_->getMapConfig(), score }`.

11. **`MapsComparison::compare`** — port HW1 scoring to the new signature
    `vector<double> compare(origin, vector<IMap3D*> targets)`. Per target:
    iterate voxel centres at origin's resolution across the intersection of
    both maps' effective bounds (from `getMapConfig()`); count
    `atVoxel(origin) == atVoxel(target)`; score = `100 * correct / total`.
    Equal resolutions assumed (bonus skipped) — mismatch throws, callers turn
    that into the -1 error path.

12. **`SimulationManager`** — verify cartesian-product loop (already flat in
    the new skeleton); wrap each `run->run()` so a thrown error becomes a
    `SimulationResult` with score -1 (and whole-group fill-in per PDF when
    e.g. a map file fails for every run in the group); populate
    `SimulationManagerReport{ generated_at_utc (UTC ISO-8601), metric =
    "output_map_accuracy", score_range = {0,100}, error_score = -1, runs }`.

13. **`SimulationRunFactoryImpl`** — wire the new deps: hidden map =
    `loadNpyArray(simulation.map_filename)` + `MapConfig{ {}, map_offset,
    map_resolution }`; output map = empty NpyArray + `MapConfig{
    mission_bounds (via LoadedComposition registry, see gap note), offset =
    bounds min corner, gps_resolution }` (Map3DImpl allocates); MockMovement
    gets `*hidden_map` + drone config; MappingAlgorithmImpl gets mission +
    drone + output MapConfig; build the hierarchical per-run output dir
    (create parent dirs) and pass `simulation, mission, output_map_file` into
    SimulationRunImpl (new ctor tail args).

14. **NEW `YamlConfigLoader.{h,cpp}`** — yaml-cpp loaders for composition /
    simulation / mission / drone / lidar configs. Returns `LoadedComposition`
    (composition data + parallel `mission_bounds`). Recoverable parsing with
    error vector (HW1 `FileParser` philosophy). Keys per updated PDF:
    `simulation_config.map_axes_offset.{x_offset,y_offset,height_offset}`,
    `mission_config.boundaries.{x,y,height}_boundary.{min_cm,max_cm}`,
    `output_mapping_resolution_factor` (int, default 1, <1 -> error+ignore),
    `drone_config.dimensions_cm`, `lidar_config.{z_min_cm,z_max_cm,d_cm,
    fov_circles}`, etc. Also a loader for the maps_comparison
    `comparison_config` (original/target: `map_res_cm`, `map_offset`,
    `map_boundaries`).

15. **NEW `SimulationReportWriter.{h,cpp}`** — yaml-cpp emitter for
    `simulation_output.yaml` per the PDF `score_report` example: writer takes
    the `SimulationManagerReport` + composition path (struct no longer carries
    it), groups the flat `runs` hierarchically by simulation_config ->
    mission_config (each `SimulationResult` carries its configs), computes the
    `summary` block (total/scored/error runs, avg/min/max). Plus a live
    per-run `error_log.txt` writer with immediate flush (so MissionControl can
    log errors the moment they occur).

16. **`drone_mapper_simulation_main.cpp`** — replace hardcoded composition:
    parse args per PDF (`[<simulation.yaml>] [<output_path>]`; missing ->
    `simulation.yaml` in cwd; bare filename / relative -> under cwd; absolute
    as-is) -> `YamlConfigLoader` -> `SimulationManager` ->
    `SimulationReportWriter`. Never `exit()`; return 0/1 from main.

17. **`maps_comparison_main.cpp`** — args: `<origin> <target>
    [comparison_config=<path>]`. No config: load both NPYs, default MapConfigs
    (same offset/bounds-from-shape, same resolution). With config: build each
    map's MapConfig from the YAML (`original`/`target` sections; enforce
    "boundaries at most map size"). Print `compare(...)[0]` to stdout (just
    the number); any error -> `-1` to stdout + message to stderr. Return 0/1.

18. **Tests** — fill `tests/components/` + `tests/integration/` (CMake glob
    already in place). One component suite per PDF list (suite names = filter
    names; SimulationRun suite also covers MockGPS + MockMovement). Two
    integration tests: real algorithm end-to-end on
    `data_maps/single_voxel_x4_y4_z4.npy`, and a gmock IMappingAlgorithm with
    a fixed command sequence. Design for the graders' mutation testing:
    component tests must fail on seeded bugs in their component (>=60%) and
    not fail on bugs elsewhere; integration catches >=20%.

19. **`readme.txt`** — document the output_results layout and
    `simulation_output.yaml` schema (PDF requirement).

20. **Final pass** — full build + tests + manual run on
    `data_maps/single_voxel_x4_y4_z4.npy`; verify maps_comparison identity
    run prints 100.

## Critical file mapping (HW1 -> HW2)
- `hw1/src/Drone.cpp` -> `hw2/src/MappingAlgorithmImpl.cpp` (biggest port:
  BFS, frontier, stuck recovery, fit check)
- `hw1/src/MockDroneMovementDriver.cpp` -> `hw2/src/MockMovement.cpp`
  (collision logic + throw -> MovementResult)
- `hw1/src/MockLidarSensor.cpp` -> verify/align `hw2/src/MockLidar.cpp`
  (skeleton ray-march now close to complete)
- `hw1/src/WorldMap.cpp` + `hw1/src/DroneMap.cpp` -> `hw2/src/Map3DImpl.cpp`
  (DONE — dense NpyArray adapter, offset-aware)
- `hw1/src/OutputWriter.cpp` (scoring half) -> `hw2/src/MapsComparison.cpp`
- `hw1/src/FileParser.cpp` -> `hw2/src/YamlConfigLoader.cpp` (KV -> YAML,
  recoverable error logging)
- `hw1/src/Simulator.cpp` + `hw1/src/main.cpp` ->
  `hw2/src/MissionControlImpl.cpp` + `hw2/src/SimulationRunImpl.cpp` +
  `hw2/src/drone_mapper_simulation_main.cpp` (orchestration split; scoring
  now in SimulationRunImpl)

## Helpers carried over
- `hw2/include/drone_mapper/VoxelKey.h` — kept for MappingAlgorithmImpl's
  internal sparse map (no longer used by Map3DImpl).
- `hw2/include/drone_mapper/ParseUtils.h` — trim/tryDouble helpers for YAML
  edge cases (still valid; the old `resolution_ratio` CLI is gone).

## Recurring constraints
- No `exit()`. Return from main.
- Errors logged immediately to error_log (PDF), never deferred.
- Failed run -> score `-1`, status error in report, composition continues;
  whole-group failures may be auto-filled with -1.
- Strong types via mp-units across all interfaces; raw doubles only inside
  impls.
- Build clean under `-Wall -Wextra -Werror -pedantic` (skeleton CMake).

## Verification
- After each step: `cmake --build build` clean (in the course devcontainer —
  vcpkg deps: mp-units>=2.3.0, tinynpy, gtest, yaml-cpp>=0.9.0).
- NOTE: steps 1-2 are code-complete but not yet compile-verified (no
  toolchain/vcpkg on this host); first build must double-check TinyNPY API
  assumptions in Map3DImpl.cpp: `NpyArray{shape, data_ptr, fortran}` ctor
  deduces int8 dtype, `Shape()`, `Type()`, `Data<T>()`, `SaveNPY(const
  char*)`/`LoadNPY(const char*)` returning error C-string.
- After step 18: `./build/drone_mapper_simulation_test` passes;
  `--gtest_filter=<Component>.*` works for every PDF-listed filter.
- After step 20:
  - `./build/drone_mapper_simulation simulation.yaml ./out/` produces
    `out/simulation_output.yaml` + populated `out/output_results/...` with
    non-`-1` scores on `data_maps/single_voxel_x4_y4_z4.npy` happy path.
  - `./build/maps_comparison a.npy a.npy` prints `100`; different maps print
    strictly between 0 and 100; `comparison_config=<path>` variant honoured.
  - Spot-check `simulation_output.yaml` shape against the PDF example.

---
---

# TEST PLAN (authoritative — written 2026-06-27, not yet implemented)

> Library + both executables compile clean under `-Wall -Wextra -Werror
> -pedantic` and run on the benchmark map (see "End-to-end status" below). This
> section enumerates every test to write. **Do not implement until reviewed.**

## Conventions & infrastructure
- One GTest/GMock executable `drone_mapper_simulation_test`, built by the
  existing `tests/CMakeLists.txt` glob over `tests/components/*.cpp` +
  `tests/integration/*.cpp`. `tests/` is on the include path; helpers live in
  `tests/test_support.h`.
- **Suite (first TEST arg) names MUST equal the PDF gtest filters exactly:**
  `SimulationManager`, `SimulationRun`, `MissionControl`, `DroneControl`,
  `MappingAlgorithm`, `MockLidar`, `MapsComparison`, `Integration`.
- **Mutation-testing design rule** (drives the whole plan):
  - A component suite must isolate its component: inject **mocks/fakes** for all
    dependencies so a seeded bug in another component cannot make it fail, and
    assert the component's own behaviour precisely (target: catch >=60% of bugs
    in its own component).
  - Integration tests use the **real wiring** and assert end-to-end outcomes
    (target: catch >=20% of bugs anywhere).
- `test_support.h` helpers (already written): `makeArray(nx,ny,nz,occupied)`
  (owning uint8, 0=air/non-zero=block), `makeConfig`, `makeBounds`, `makeDrone`,
  `makeLidar`, `makeMission`, `posCm`, `orient`. **To add:** `writeTempNpy(...)`
  (save an array via `NpyArray::SaveNPY` to a unique temp path) and a
  `dataMapPath("benchmark_map.npy")` helper. Add a CMake compile definition
  `DRONE_MAPPER_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data_maps"` so integration
  tests can locate `data_maps/` regardless of cwd.
- GMock interfaces to define (in test files / a shared header): `IMappingAlgorithm`
  (note: ctor takes mission+lidar+drone+`const IMap3D&`; use
  `using IMappingAlgorithm::IMappingAlgorithm;` + `MOCK_METHOD nextStep`),
  `IDroneControl`, `IDroneMovement`, `IMissionControl`, `ISimulationRunFactory`,
  `ISimulationRun` (+ a small stub run returning a fixed `SimulationResult`).
  Prefer `NiceMock<>` and `ON_CALL` defaults; `EXPECT_CALL` only where asserting
  call sequence/count.

## Component suite: MapsComparison  (tests/components/maps_comparison_test.cpp — DRAFTED)
1. IdenticalMapsReturn100 — same array+config twice -> 100.
2. OppositeMapsReturnZero — all-empty vs all-occupied -> 0.
3. SimilarMapsAreHighButNotPerfect — one differing voxel -> (90,100).
4. DifferentResolutionsThrow — origin/target resolution mismatch -> throws (bonus skipped).
5. MultipleTargetsProduceMultipleScores — 2 targets -> 2 scores, order preserved.
6. PartialOverlapBounds — target bounds offset so only part overlaps -> scored on the intersection only.
7. UnmappedCountsAsWrong — int8 output with `-1` cells vs hidden Empty -> score < 100 (Unmapped != Empty).
8. PotentiallyOccupiedCountsAsWrong — output `-3` vs hidden Occupied/Empty -> mismatch.

## Component suite: MockLidar  (tests/components/mock_lidar_test.cpp)
Fixtures: a small hidden `Map3DImpl` with a known Occupied wall; `MockGPS` at a known pose; `MockLidar(makeLidar(...), map, gps)`.
1. ReportsObstacleAheadWithinRange — wall ahead -> centre-beam distance in (z_min, z_max).
2. MissReturnsMaxDistance — no obstacle -> distance == max double.
3. TooCloseHitReturnsZero — wall closer than z_min -> distance == 0.
4. ZeroFovCirclesGivesEmptyScan — fov_circles=0 -> empty result.
5. BeamCountMatchesFovCircles — fov=N -> 1+4+16+... beams (sum of 4^k).
6. ScanIsRelativeToHeading — same wall, two GPS headings -> the hit appears on the beam whose (relative angle + heading) points at the wall (verifies heading is added once).
7. ConfigGetterReturnsInjectedConfig — config() round-trips the ctor value.
(This suite "verifies the mock's correct implementation" per the PDF.)

## Component suite: MappingAlgorithm  (tests/components/mapping_algorithm_test.cpp)
Fixtures: a `Map3DImpl` **output** map the test pre-populates with `set(...)` to
stage the drone's "knowledge"; `MappingAlgorithmImpl(mission, lidar, drone, output)`.
Drone `DroneState` is supplied directly; for multi-step navigation, drive a loop
that applies the returned `MovementCommand`s to a `MockGPS`/`MockMovement` and
feeds the updated state back (keeps the algorithm isolated from the real lidar).
1. FirstStepRequestsScan — fresh algo -> first `nextStep` returns a `scan_orientation`, status Working.
2. ScansSixDirectionsBeforeMoving — stationary drone -> first six commands are the six scan orientations, no movement.
3. FinishesWhenNothingNavigable — output all Unmapped -> after scans+expand, status Finished.
4. NavigatesIntoEmptyNeighbour — pre-set an Empty, body-fitting corridor -> algo emits Rotate/Advance toward it.
5. OnlyEmptyIsPassable — neighbour Occupied or Unmapped -> not navigated (no move into it).
6. DroneSizeGatesNarrowGaps — corridor 2 voxels wide: large drone (radius> gap) -> can't fit (Finished/avoids); small drone -> navigates. (canFitAt sphere check.)
7. MovementCommandsRespectLimits — every emitted Rotate<=max_rotate, Advance<=max_advance, Elevate<=max_elevate.
8. ReachesFarCellViaBfs — long staged Empty corridor -> driven loop reaches the far end.
9. ElevateUsedForVerticalNeighbour — staged Empty cell directly above -> an Elevate command is emitted.

## Component suite: SimulationRun (+ MockGPS + MockMovement)  (tests/components/simulation_run_test.cpp)
MockGPS / MockMovement (PDF: SimulationRun suite also tests these):
1. MockGps_ReturnsAndUpdatesPose — getters + setPosition/setHeading.
2. MockMovement_AdvancesAlongHeading — east -> +x by the distance.
3. MockMovement_RotateUpdatesHeading — Left=+, Right=-, normalised to [0,360).
4. MockMovement_ElevateSignedZ — positive/negative elevate moves z.
5. MockMovement_ClampsToMax — advance/elevate/rotate beyond max are clamped.
6. MockMovement_AdvanceAfterRotate — rotate 90 then advance -> +y.
SimulationRun proper (build via `SimulationRunFactoryImpl` on a `writeTempNpy` map, or with mocked deps):
7. RunReturnsResultWithConfigs — run() carries the sim+mission configs, one mission_result, output_map_file set, output_map_config populated.
8. RunScoresCompletedRunInRange — happy path -> score in [0,100], status completed/max_steps.
9. RunScoresErrorAsMinusOne — mission_control (mock) returns Error -> SimulationResult.mission_score == -1.
10. ResolutionStatusFromFactor — factor<1 -> IgnoredTooSmall; ==1 -> Accepted; >1 -> Ignored.

## Component suite: MissionControl  (tests/components/mission_control_test.cpp)
Fixtures: `NiceMock<IDroneControl>` for step()/state(); real hidden+output `Map3DImpl`; temp output path.
1. CompletesWhenDroneFinishes — step()-> Continue then Completed -> status Completed; output_map.npy saved.
2. MaxStepsWhenNeverCompletes — step() always Continue -> status MaxSteps, steps==max_steps.
3. DroneStepErrorPropagates — step()-> Error -> status Error, ErrorRef code DRONE_STEP_FAILED.
4. DetectsCollisionWhileMoving — state() moves the drone into a hidden Occupied region -> Error, DRONE_HITS_OBSTACLE.
5. DetectsInitialPositionInsideObstacle — start pose inside a block -> Error at step 0, DRONE_HITS_OBSTACLE.
6. ReportsInvalidMissionBounds — mission_bounds max<=min -> Error, MISSION_BOUNDARY_INVALID, steps 0, (no save).
7. ErrorLoggedImmediately — after an error, error_log.txt in the run dir contains the code (written, not deferred).
8. NoCollisionInOpenMap — empty hidden map, drone wanders -> never DRONE_HITS_OBSTACLE.

## Component suite: DroneControl  (tests/components/drone_control_test.cpp)
Fixtures: `NiceMock<IMappingAlgorithm>` (scripted commands) + real MockGPS, MockMovement, MockLidar, output Map3DImpl. (May also use `NiceMock<IDroneMovement>` to force a movement failure.)
1. ScanCommandAppliesToOutputMap — algo returns a scan -> output map gains Empty/Occupied cells; returns Continue.
2. MoveCommandMovesDrone — algo returns Advance -> GPS position advances; Continue.
3. MovementFailureReturnsError — IDroneMovement returns {false,...} -> step() Error with the message.
4. FinishedCommandReturnsCompleted — algo status Finished -> step() Completed.
5. MarksBodyFootprintEmpty — after a step the drone's spherical footprint is Empty in the output map.
6. StateReflectsGpsAndStepIndex — state() == {gps pose, step count}; step index increments per step.
7. LatestScanFedBack — the scan performed this step is passed as `latest_scan` to the next nextStep (capture via the mock).

## Component suite: SimulationManager  (tests/components/simulation_manager_test.cpp)
Fixtures: `NiceMock<ISimulationRunFactory>` whose create() returns a stub `ISimulationRun` (fixed SimulationResult).
1. RunsCartesianProduct — composition with m missions x d drones x l lidars -> create() called m*d*l times; report.runs size matches.
2. ReportMetadata — metric=="output_map_accuracy", score_range=={0,100}, error_score==-1, generated_at_utc non-empty/ISO-8601-ish.
3. FactoryThrowBecomesMinusOneResult — create()/run() throws -> that run is a SimulationResult with score -1, status Error (RUN_FAILED); other runs unaffected.
4. NestedGroupsIterated — 2 simulation_mission_groups -> runs produced for both, each carrying its own configs.
5. PerRunOutputDirsUnique — create() receives a distinct output_path per (sim,mission,drone,lidar) (assert via captured args).

## Integration suite  (tests/integration/integration_test.cpp)
Real wiring (real factory/algorithm) unless stated. Use `DRONE_MAPPER_DATA_DIR`.
1. RealAlgorithm_SingleVoxel_EndToEnd — `single_voxel_x4_y4_z4.npy` via SimulationManager+real factory: 1 run, status completed, score >= 0 (not -1), output_map.npy written and numpy-readable (int8), simulation_output.yaml well-formed.
2. RealAlgorithm_Benchmark_MapsAndScores — `benchmark_map.npy`, small drone (dimensions 20), start at a verified empty interior cell, generous max_steps: completes within the time budget, **no collision**, mapped (non-Unmapped) cells above a threshold (e.g. > 8000), score > 0. (Staff-recommended.)
3. DroneSizeChangesCoverage_Benchmark — same map+start, run a SMALL drone (dimensions 20, fits 2x2 rooms) and a LARGE drone (dimensions 30, only fits 3x3+ entrances); assert the small drone maps strictly MORE cells / reaches at least one room the large drone does not. (Exercises the map's 3x3 / 2x2 / 1x1 room-entrance feature.)
4. MockAlgorithm_EndToEnd — `NiceMock<IMappingAlgorithm>` scripted (a few scans, a couple of moves, then Finished) wired through real DroneControl+MissionControl+MockGPS/Movement/Lidar on a small hidden map: mission completes, the scripted scans appear in the output map, GPS ended where the script drove it. (PDF integration test #2 "with a mock algorithm".)
5. MockAlgorithm_CollisionReported — script the drone straight into a wall -> end-to-end DRONE_HITS_OBSTACLE, run score -1, error_log written, composition still returns a report.
6. WholeGroupMapFailure_FilledMinusOne — composition referencing a missing/…bad map file -> all runs in that group scored -1, others succeed.
7. OutputMapMatchesReportedScore — after run #1, load the produced output_map.npy + hidden map and call MapsComparison -> equals the score in simulation_output.yaml (consistency of the pipeline).

## End-to-end status (already verified manually, informs the integration tests)
- benchmark_map.npy (29x30x31, uint8 Minecraft block ids; ground z=0..14 solid,
  house z=15..27): drone start voxel (5,5,17) = world (55,55,175) at res 10cm,
  mission bounds [0,290)x[0,300)x[0,310), drone dimensions 30, lidar fov 4.
- With sufficient max_steps the run **completes** in ~14.6s, maps ~14k/27k cells
  (~95% of reachable air + wall surfaces), score ~52, no collision. With
  max_steps 20000 it hits MaxSteps (~8s, score ~29). Integration tests should
  set max_steps generously and assert coverage/no-collision/time, not an exact score.
