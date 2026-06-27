# HW2 Handoff

> Snapshot for resuming in a fresh session. Last updated 2026-06-10.
> Full roadmap + rationale lives in [PLAN.md](PLAN.md) — read it alongside this.

## What this project is
Refactor of the HW1 drone-mapping simulator onto the course-supplied ex2
skeleton. Locked interfaces (`I*` headers, `types::` structs), new I/O (`.npy`
maps + YAML configs), GTest/GMock component+integration tests, hierarchical
YAML output report, immediate error logging. Working tree: `cpp/hw2/`. Source
of logic to port: `cpp/hw1/` (never modify). Updated skeleton the user
re-extracts: `cpp/ex_2_skeleton-main/`. Assignment PDF + CLAUDE.md design rules
are in-tree (hw1/CLAUDE.md holds the 23 HW1 design rules — still the behavioural
spec for ported logic).

## Working method (IMPORTANT)
- **One file (its .h + .cpp together) per step**, then STOP and wait for the
  user's go-ahead before the next file. Do not run ahead.
- **No anonymous-namespace helpers.** Helper functions go in the class header
  `private:` section (static if stateless), camelCase to match members.
  File-local constants become `static constexpr` private members. Lambdas in a
  function body are fine. (Applied to all files below.)
- **Cannot compile on this host** — no vcpkg/toolchain here. Build verification
  happens on the user's side in the course devcontainer
  (`cmake --build build`). Keep the tree compile-ready; the user checks.

## Progress
| Step | File | Status |
|------|------|--------|
| 1 | Bootstrap (CMakeLists, vcpkg.json, tests/ subdir) | DONE |
| 2 | Map3DImpl (.h/.cpp) | DONE |
| 3 | MockGPS (.h/.cpp) | DONE — skeleton already complete, no change |
| 4 | MockMovement (.h/.cpp) | DONE |
| 5 | MockLidar (.h/.cpp) | DONE |
| 6 | ScanResultToVoxels (.cpp) | **NEXT** |
| 7 | MappingAlgorithmImpl (.h/.cpp) | pending |
| 8 | DroneControlImpl (.cpp) | pending |
| 9 | MissionControlImpl (.cpp) | pending |
| 10 | SimulationRunImpl (.cpp) | pending |
| 11 | MapsComparison (.cpp) | pending |
| 12 | SimulationManager (.cpp) | pending |
| 13 | SimulationRunFactoryImpl (.cpp) | pending — fixes compile-blocker |
| 14 | YamlConfigLoader (.h/.cpp NEW) | pending |
| 15 | SimulationReportWriter (.h/.cpp NEW) | pending |
| 16 | drone_mapper_simulation_main.cpp | pending |
| 17 | maps_comparison_main.cpp | pending |
| 18 | Tests (components + integration) | pending |
| 19 | readme.txt | pending |
| 20 | Final build + run | pending |

## Known compile-blocker (expected, not a bug)
The library will NOT compile until step 13 because
[src/SimulationRunFactoryImpl.cpp](src/SimulationRunFactoryImpl.cpp) still
calls the old `MockMovement(*gps)` ctor. MockMovement's ctor is now
`(MockGPS&, const IMap3D& world_map, types::DroneConfigData)`. This is the only
introduced break so far; it gets rewired when we do the factory. The individual
done files (Map3DImpl, MockGPS, MockMovement, MockLidar) are each internally
consistent with the locked types.

## What was done in steps 2–5 (so a resumer trusts the state)
- **Map3DImpl** — dense `shared_ptr<NpyArray>` + `MapConfig` (boundaries,
  offset, resolution). World→index: `floor((pos - offset)/res)`, cell (0,0,0)
  at `offset`, C-order flat `(ix*ny+iy)*nz+iz`. Loaded maps validate 3-D shape
  + dtype (`u`/`b` uint8, `i` int8; 1=Occupied, 0=Empty, -1=Unmapped, uint8
  reinterpreted as int8). Empty array + real bounds ⇒ allocates dense int8 grid
  over `[offset, bounds max)` filled Unmapped (output-map mode). `atVoxel`:
  outside bounds/footprint→OutOfBounds; inside bounds but past array→Empty
  (hidden-map semantics); else raw value. `save` = `SaveNPY` as-is. Helpers are
  private static members; `GridGeometry` is a private nested struct.
- **MockGPS** — unchanged; already has ctor, getters, `setPosition`/`setHeading`.
- **MockMovement** — HW1 `MockDroneMovementDriver` port. Ctor extended to take
  the world map + drone config; resolution read from
  `world_map.getMapConfig().resolution`. `rotate/advance/elevate` return
  `MovementResult{success,msg}` (no throws). Body-sweep collision identical to
  HW1, sphere half-extent = `dimensions/2`. **Front-cap fix applied**: the path
  loop sweeps from `step_cm` to `abs_dist + radius` (replacing the old
  center-to-center loop + separate endpoint check), so the front of the sphere
  is tested; rear hemisphere intentionally skipped (already-cleared start). The
  5 tuning constants are private `static constexpr` members.
- **MockLidar** — verified against HW1 `MockLidarSensor`; kept skeleton's ring
  radius `circle * d` (ex2 redefines `d` as inter-circle spacing), `==Occupied`
  hit test, miss = max-double, `fov_circles==0`→empty, relative-angle storage.
  Aligned two things to HW1: beam march **starts at `step`** (not 0) and uses
  **`0.2 * resolution`** (`kStepFraction`, was 0.1). Helpers (`beamsOnCircle`,
  `horizontalDelta`, `altitudeDelta`) are private static members.

## Design decisions to remember (the non-obvious ones)
- **Mission boundaries gap**: mission YAML still has `boundaries` but the locked
  `MissionConfigData` dropped them and `ISimulationRunFactory::create(...)` is
  fixed. Plan: YAML loader returns `LoadedComposition { SimulationCompositionData
  data; std::vector<MappingBounds> mission_bounds; }`; main hands the factory the
  composition + bounds; factory recovers the mission index by pointer arithmetic
  (`&mission - &composition.missions[0]`). See PLAN.md "Mission boundaries gap".
- **Scoring moved** out of MissionControl into SimulationRunImpl (new types).
- **No bonuses** — including the new different-resolution maps_comparison bonus,
  so mismatched resolutions are an error path, not a feature.
- **Resolution policy**: always use input resolution; every run reports
  `resolution_request_status = Ignored`. `output_mapping_resolution_factor`:
  int, default 1, `<1` → error log + Ignored.
- **Output layout**:
  `<out>/output_results/<sim_stem>/<mission_stem>/<drone_stem>__<lidar_stem>/{output_map.npy,error_log.txt}`
  + `<out>/simulation_output.yaml`.
- **Test suite names must equal the PDF gtest filters**: SimulationManager,
  SimulationRun, MissionControl, DroneControl, MappingAlgorithm, MockLidar,
  MapsComparison, Integration.

## Build (on the user's devcontainer, not here)
vcpkg deps: mp-units>=2.3.0, tinynpy, gtest, yaml-cpp>=0.9.0 (all in
[vcpkg.json](vcpkg.json)). Configure with the vcpkg toolchain, then
`cmake --build build`. First green build should sanity-check the TinyNPY API
assumptions in Map3DImpl.cpp (`NpyArray{shape, data_ptr, fortran}` deduces int8
dtype; `Shape()`, `Type()`, `Data<T>()`, `LoadNPY`/`SaveNPY(const char*)`
returning an error C-string). Run tests: `./build/.../drone_mapper_simulation_test`,
filterable by `--gtest_filter=<Component>.*`.

## Next action
Start **step 6: ScanResultToVoxels** — port HW1 `Drone::processBeam` to
[src/ScanResultToVoxels.cpp](src/ScanResultToVoxels.cpp) (`.h` is locked). Per
hit: abs beam = heading + hit.angle; hit (`distance <= z_max`) marches sub-voxel
marking intermediate voxels Empty + final Occupied; miss (max-double) marks the
full path Empty; `distance == 0` (too-close) marks nothing. Static signature has
no resolution param → use ~1 cm fallback step. Then STOP for review.
