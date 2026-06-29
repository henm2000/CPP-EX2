cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# Default to an optimized build when no configuration was requested. mp-units is
# heavily templated, so an unoptimized build runs ~20x slower at runtime (every
# quantity conversion in the atVoxel / lidar-trace hot loops becomes a real
# call). Only applies to single-config generators (Makefiles/Ninja).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


find_package(mp-units CONFIG REQUIRED)
# The vcpkg tinynpy port ships its config as TinyNPYConfig.cmake; accept either
# spelling so the build works regardless of the installed port's casing.
find_package(TinyNPY CONFIG QUIET)
if(NOT TinyNPY_FOUND)
    find_package(tinynpy CONFIG REQUIRED)
endif()
find_package(yaml-cpp REQUIRED) # new!

add_library(drone_mapper
    src/DroneControlImpl.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
    src/YamlConfigLoader.cpp
    src/SimulationReportWriter.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
    PRIVATE
        yaml-cpp::yaml-cpp
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

# Compile the yaml example file
add_executable(example_yml cpp_yaml_example/main.cpp)
target_link_libraries(example_yml PRIVATE yaml-cpp::yaml-cpp)

enable_testing()
add_subdirectory(tests)
