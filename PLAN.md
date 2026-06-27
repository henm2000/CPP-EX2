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
