================================================================================
 drone_mapper - Advanced Topics in Programming, Assignment 2
================================================================================

A drone-mapping simulator. A simulated drone explores a hidden voxel world using
a mock LiDAR, builds an output occupancy map, and is scored by comparing the
output map against the hidden ground-truth map. Many (simulation x mission x
drone x lidar) combinations are run from a single composition file and scored
into one hierarchical report.

--------------------------------------------------------------------------------
1. BUILDING
--------------------------------------------------------------------------------
Dependencies (mp-units, TinyNPY, yaml-cpp, GTest) are resolved with vcpkg.

Option A - CMake presets (devcontainer / Ninja):
    cmake --preset default
    cmake --build --preset default

Option B - plain CMake + a local vcpkg toolchain:
    cmake -S . -B build -G "Unix Makefiles" \
          -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
    cmake --build build -j

The build defaults to an optimized (Release) configuration when no build type is
given; this matters because the mp-units quantity math in the per-voxel hot loops
is ~20x slower unoptimized.

Build targets:
    drone_mapper_simulation        - the simulator executable
    maps_comparison                - standalone map-vs-map scorer
    drone_mapper_simulation_test   - the GTest/GMock test suite

--------------------------------------------------------------------------------
2. RUNNING THE SIMULATOR
--------------------------------------------------------------------------------
    ./drone_mapper_simulation [<composition.yaml>] [<output_path>]

Argument handling:
    - no first argument        -> uses "simulation.yaml" in the current directory
    - bare filename            -> resolved under the current directory
    - relative path            -> resolved under the current directory
    - absolute path (/...)      -> used as-is
    - no second argument       -> output is written under the current directory

The program never calls exit(); it always returns from main. A composition that
cannot be loaded prints to stderr and returns 1.

--------------------------------------------------------------------------------
3. RUNNING THE MAP COMPARISON
--------------------------------------------------------------------------------
    ./maps_comparison <origin_map.npy> <target_map.npy> [comparison_config=<path>]

Prints the accuracy (0..100) of the target map versus the origin map.

--------------------------------------------------------------------------------
4. RUNNING THE TESTS
--------------------------------------------------------------------------------
    ./drone_mapper_simulation_test                              (all tests)
    ./drone_mapper_simulation_test --gtest_filter=Integration.*
    ./drone_mapper_simulation_test --gtest_filter=SimulationManager.*
    ./drone_mapper_simulation_test --gtest_filter=SimulationRun.*
    ./drone_mapper_simulation_test --gtest_filter=MissionControl.*
    ./drone_mapper_simulation_test --gtest_filter=DroneControl.*
    ./drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
    ./drone_mapper_simulation_test --gtest_filter=MockLidar.*
    ./drone_mapper_simulation_test --gtest_filter=MapsComparison.*

Component suites isolate one component with GMock doubles for its dependencies;
the Integration suite wires the real components end-to-end. The two required
integration tests are RealAlgorithm_* (real mapping algorithm) and MockAlgorithm_*
(scripted mock algorithm).

--------------------------------------------------------------------------------
5. INPUT FILE FORMAT (the composition file)
--------------------------------------------------------------------------------
The composition file is the top-level input. It references separate per-section
config files (relative paths are resolved next to the composition file):

    simulation_compositions:
      simulations:
        - simulation_config: sim0.yaml
          mission_configs: [mission0.yaml, mission1.yaml]
      drone_configs: [drone0.yaml]
      lidar_configs: [lidar0.yaml]

The simulator runs the full cartesian product:
    (each mission of each simulation) x (each drone) x (each lidar).

Each referenced file holds one section (all keys optional; sensible defaults are
used when a key is missing):

  simulation_config:                # the hidden ground-truth world
    map_filename: map.npy           # .npy voxel map; uint8 (0=air, nonzero=block)
                                    #   or int8 are both accepted
    map_resolution_cm: 10           # cm per voxel
    initial_angle_deg: 0            # drone start heading
    map_axes_offset: { x_offset: 0, y_offset: 0, height_offset: 0 }
    initial_drone_position: { x_cm: 55, y_cm: 55, height_cm: 175 }

  mission_config:
    max_steps: 40000
    gps_resolution_cm: 10           # resolution the output map is built at
    output_mapping_resolution_factor: 1.0   # <1 is ignored + logged; see notes
    boundaries:
      x_boundary:      { min_cm: 0, max_cm: 290 }
      y_boundary:      { min_cm: 0, max_cm: 300 }
      height_boundary: { min_cm: 0, max_cm: 310 }

  drone_config:                     # the drone is a sphere of diameter dimensions_cm
    dimensions_cm: 30               # diameter; radius = dimensions_cm / 2
    max_rotate_deg: 45              # per-command limits
    max_advance_cm: 50
    max_elevate_cm: 40

  lidar_config:                     # see ex1 for the FOV-circle beam pattern
    z_min_cm: 20
    z_max_cm: 120
    d_cm: 2.5
    fov_circles: 4

--------------------------------------------------------------------------------
6. OUTPUT FORMAT
--------------------------------------------------------------------------------
In the output path the simulator writes (overwriting existing files):

  simulation_output.yaml      the score report (see layout below)
  output_results/             all per-run output maps and error logs

6a. output_results/ folder layout
Each run gets its own nested directory, named by indices into the composition:

  output_results/
    sim<G>_<mapstem>/         G = simulation group index, mapstem = map filename stem
      mission<M>/             M = mission index within that simulation
        drone<D>__lidar<L>/   D = drone index, L = lidar index
          output_map.npy      the produced int8 occupancy map
          error_log.txt       present only if that run logged an error

  error_log.txt lines have the form "<CODE>: <message>", appended the moment the
  error occurs. Codes: MISSION_BOUNDARY_INVALID, DRONE_HITS_OBSTACLE,
  DRONE_STEP_FAILED, OUTPUT_SAVE_FAILED, RESOLUTION_TOO_SMALL, RUN_FAILED.

6b. simulation_output.yaml layout
Runs are regrouped hierarchically (simulation -> mission -> run) with a summary:

  score_report:
    composition_file: <path to the composition file>
    generated_at_utc: 2026-06-29T12:20:12Z      # ISO-8601 UTC
    metric: output_map_accuracy
    score_range: { min: 0, max: 100 }
    error_score: -1                              # the score used for failed runs
    summary:
      total_runs: 1
      scored_runs: 1                             # runs with a non-negative score
      error_runs: 0
      average_score: 100                         # over scored runs only
      min_score: 100
      max_score: 100
    simulations:
      - simulation_map: <hidden map path>
        map_resolution_cm: 10
        missions:
          - max_steps: 3000
            resolution_cm: 10
            resolution_request_status: ACCEPTED  # ACCEPTED | IGNORED | IGNORED_TOO_SMALL
            runs:
              - output_map: <path to output_map.npy>
                status: completed                # completed | max_steps | error
                steps: 1228
                score: 100                       # accuracy 0..100, or -1 on error

--------------------------------------------------------------------------------
7. NOTES
--------------------------------------------------------------------------------
- Score = percentage of overlapping voxel centres where the output map and the
  hidden map agree. Unmapped (-1) and PotentiallyOccupied (-3) output voxels count
  as mismatches. A run whose mission errors is scored -1.
- output_mapping_resolution_factor: only factor == 1 is honoured. A factor < 1
  (finer than GPS) is impossible, so it is ignored and a RESOLUTION_TOO_SMALL line
  is logged; a factor > 1 is ignored silently. The output map is always built at
  the GPS resolution.
- The drone is modelled as a sphere. It only enters voxels its whole body fits
  inside (all body voxels Empty), so smaller drones reach through narrower room
  entrances (1x1 / 2x2 / 3x3 cells) than larger drones.
- Maps may be uint8 (input convention: 0 = air, any nonzero block id = Occupied)
  or int8 (output/sentinel convention: -1 Unmapped, -2 OutOfBounds,
  -3 PotentiallyOccupied, 0 Empty, otherwise Occupied). Both are supported.
