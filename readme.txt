drone_mapper (Assignment 2) — output formats and run instructions
=================================================================

This document describes (a) how to run the two executables, (b) the layout of
the `output_results/` folder, and (c) the schema of `simulation_output.yaml`,
as required by the assignment.


Building
--------
The project uses vcpkg + CMake (see CMakePresets.json). In the course
devcontainer:

    cmake --preset default
    cmake --build build

Dependencies (vcpkg.json): mp-units (>=2.3.0), yaml-cpp (>=0.9.0), tinynpy,
gtest. The library is built with -Wall -Wextra -Werror -pedantic.


Running the simulation
----------------------
    ./drone_mapper_simulation [<simulation.yaml>] [<output_path>]

  - <simulation.yaml> : the composition file. If omitted, "simulation.yaml" in
    the current working directory is used. A bare filename or a relative path is
    resolved under the current working directory; an absolute path is used as-is.
  - <output_path>     : where outputs are written. If omitted, the current
    working directory is used.

Referenced config paths inside the composition file (simulation/mission/drone/
lidar yaml files, and each simulation's map_filename) are resolved relative to
the composition file's directory when they are relative.

The program never calls exit(); it always returns 0 (success) or 1 (the
composition file could not be read).


Running the maps comparison utility
------------------------------------
    ./maps_comparison <origin_map> <target_map> [comparison_config=<path>]

  - Prints a single accuracy number in [0, 100] to standard output (nothing
    else). Two identical maps print 100.
  - On any error it prints -1 to standard output and a descriptive message to
    standard error, and returns 1.
  - Without a comparison_config, both maps are assumed to share the same offset
    (0,0,0), resolution (1 cm cell-for-cell), and their own array footprint as
    bounds. With a comparison_config, each map's resolution / offset / bounds are
    taken from the file (different-resolution comparison is an optional bonus and
    is reported as an error here).


Output: output_results/ folder layout
--------------------------------------
For every single run (one cell of the cartesian product
[missions] x [drones] x [lidars] within each simulation), the program writes a
directory:

    <output_path>/output_results/
        sim<G>_<map-stem>/            # G = simulation-group index, map-stem = npy stem
            mission<M>/               # M = mission index within that simulation
                drone<D>__lidar<L>/   # D = drone index, L = lidar index
                    output_map.npy    # the drone's produced map (.npy, int8)
                    error_log.txt     # created only if errors occurred in the run

  - output_map.npy is a 3-D int8 NumPy array over the mission bounds at the GPS
    resolution. Cell values: 1 = Occupied, 0 = Empty, -1 = Unmapped,
    -3 = PotentiallyOccupied (a hit closer than the lidar's z_min, exact cell
    unknown).
  - error_log.txt contains one "CODE: message" line per error, written and
    flushed the moment the error occurs (never deferred). Error codes used:
    MISSION_BOUNDARY_INVALID, DRONE_HITS_OBSTACLE, DRONE_STEP_FAILED,
    OUTPUT_SAVE_FAILED, RUN_FAILED.

The (drone, lidar) identity of each run is encoded in its folder name
(drone<D>__lidar<L>), matching the index order of drone_configs / lidar_configs
in the composition file.


Output: simulation_output.yaml schema
-------------------------------------
Written to <output_path>/simulation_output.yaml. The flat list of runs is
regrouped hierarchically by simulation -> mission -> run:

    score_report:
      composition_file: "<path to the composition yaml>"
      generated_at_utc: "YYYY-MM-DDThh:mm:ssZ"   # ISO-8601 UTC
      metric: "output_map_accuracy"
      score_range:
        min: 0
        max: 100
      error_score: -1
      summary:
        total_runs:  <int>
        scored_runs: <int>      # runs with a non-negative score
        error_runs:  <int>      # runs scored -1
        average_score: <double> # over scored runs
        min_score: <double>
        max_score: <double>
      simulations:
        - simulation_map: "<map .npy path>"
          map_resolution_cm: <double>
          missions:
            - max_steps: <int>
              resolution_cm: <double>                    # the actual output resolution
              resolution_request_status: ACCEPTED | IGNORED | IGNORED TOO SMALL
              runs:
                - output_map: "<path to this run's output_map.npy>"
                  status: "completed" | "max_steps" | "error"
                  steps: <int>
                  score: <double>          # 0..100, or -1 on error
                  error_ref:               # present only for error runs
                    code: "<error code>"
                    message: "<description>"

Notes on scoring and policy
---------------------------
  - Score = percentage of voxel centres (sampled at the map resolution over the
    overlap of the hidden and output maps' bounds) where the produced map agrees
    with the ground-truth map. A failed run scores -1 and the composition
    continues; a whole group whose map file cannot be loaded is filled with -1.
  - Resolution policy: the simulation always maps at the GPS resolution.
    output_mapping_resolution_factor < 1 -> IGNORED TOO SMALL (logged); == 1 ->
    ACCEPTED; > 1 -> IGNORED.


Tests
-----
    ./drone_mapper_simulation_test
    ./drone_mapper_simulation_test --gtest_filter=<Suite>.*

Component suites: SimulationManager, SimulationRun (also covers MockGPS and
MockMovement), MissionControl, DroneControl, MappingAlgorithm, MockLidar,
MapsComparison. Integration suite: Integration.
