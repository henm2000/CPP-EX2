#pragma once

// Shared helpers for the drone_mapper component + integration tests.

#include <TinyNPY.h>

#include <drone_mapper/Types.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_mapper::test_support {

// Builds a dense uint8 input-style occupancy array (0 = air/Empty, non-zero =
// Occupied block) of the given shape, marking the listed (ix, iy, iz) cells
// Occupied. Uses the explicit-dtype constructor so the array owns its buffer.
inline std::shared_ptr<NpyArray> makeArray(std::size_t nx, std::size_t ny, std::size_t nz,
                                           const std::vector<std::array<std::size_t, 3>>& occupied = {}) {
    auto array = std::make_shared<NpyArray>(std::vector<std::size_t>{nx, ny, nz},
                                            sizeof(std::uint8_t), 'u', false);
    array->Allocate();
    std::uint8_t* data = array->Data<std::uint8_t>();
    std::fill(data, data + array->NumValue(), static_cast<std::uint8_t>(0));
    for (const auto& c : occupied) {
        data[(c[0] * ny + c[1]) * nz + c[2]] = 1;
    }
    return array;
}

inline Position3D posCm(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

inline Orientation orient(double horizontal_deg, double altitude_deg = 0.0) {
    return Orientation{horizontal_deg * horizontal_angle[deg], altitude_deg * altitude_angle[deg]};
}

inline types::MappingBounds makeBounds(double x0, double x1, double y0, double y1, double z0, double z1) {
    return types::MappingBounds{
        x0 * x_extent[cm], x1 * x_extent[cm],
        y0 * y_extent[cm], y1 * y_extent[cm],
        z0 * z_extent[cm], z1 * z_extent[cm],
    };
}

inline types::MapConfig makeConfig(const types::MappingBounds& bounds, double res_cm,
                                   const Position3D& offset = {}) {
    return types::MapConfig{bounds, offset, res_cm * cm};
}

inline types::DroneConfigData makeDrone(double radius_cm = 5.0, double max_rotate_deg = 90.0,
                                        double max_advance_cm = 50.0, double max_elevate_cm = 50.0) {
    return types::DroneConfigData{
        radius_cm * cm,
        max_rotate_deg * horizontal_angle[deg],
        max_advance_cm * cm,
        max_elevate_cm * cm,
    };
}

inline types::LidarConfigData makeLidar(double z_min_cm = 20.0, double z_max_cm = 200.0,
                                        double d_cm = 2.5, std::size_t fov_circles = 1) {
    return types::LidarConfigData{z_min_cm * cm, z_max_cm * cm, d_cm * cm, fov_circles};
}

inline types::MissionConfigData makeMission(const types::MappingBounds& bounds,
                                            std::size_t max_steps = 500, double gps_res_cm = 10.0,
                                            double resolution_factor = 1.0) {
    types::MissionConfigData mission{};
    mission.max_steps = max_steps;
    mission.gps_resolution = gps_res_cm * cm;
    mission.output_mapping_resolution_factor = resolution_factor;
    mission.mission_bounds = bounds;
    return mission;
}

// Saves an array to a unique temp .npy file and returns its path, so tests that
// need a real on-disk map (SimulationRun factory, integration) can produce one.
inline std::filesystem::path writeTempNpy(const std::shared_ptr<NpyArray>& array) {
    static std::atomic<unsigned long long> counter{0};
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "drone_mapper_tests";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::filesystem::path path =
        dir / ("map_" + std::to_string(counter.fetch_add(1)) + "_" +
               std::to_string(reinterpret_cast<std::uintptr_t>(array.get())) + ".npy");
    if (const char* err = array->SaveNPY(path.string())) {
        throw std::runtime_error(std::string{"writeTempNpy: "} + err);
    }
    return path;
}

// Absolute path to a map shipped under tests/data_maps via the
// DRONE_MAPPER_DATA_DIR compile definition (set by tests/CMakeLists.txt).
inline std::filesystem::path dataMapPath(const std::string& name) {
#ifdef DRONE_MAPPER_DATA_DIR
    return std::filesystem::path{DRONE_MAPPER_DATA_DIR} / name;
#else
    return std::filesystem::path{"data_maps"} / name;
#endif
}

} // namespace drone_mapper::test_support
