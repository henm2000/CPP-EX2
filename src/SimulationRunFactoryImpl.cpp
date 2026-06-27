#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <memory>
#include <stdexcept>
#include <string>

namespace drone_mapper {

std::shared_ptr<NpyArray> SimulationRunFactoryImpl::loadNpyArray(const std::filesystem::path& path) {
    auto array = std::make_shared<NpyArray>();
    const char* error = array->LoadNPY(path.string().c_str());
    if (error != nullptr) {
        throw std::runtime_error("failed to load map '" + path.string() + "': " + error);
    }
    return array;
}

types::MapConfig SimulationRunFactoryImpl::hiddenMapConfig(const NpyArray& array,
                                                           const types::SimulationConfigData& simulation) {
    types::MapConfig config{};
    config.offset = simulation.map_offset;
    config.resolution = simulation.map_resolution;

    const auto& shape = array.Shape();
    if (shape.size() >= 3) {
        const double res = simulation.map_resolution.force_numerical_value_in(cm);
        const double ox = simulation.map_offset.x.force_numerical_value_in(cm);
        const double oy = simulation.map_offset.y.force_numerical_value_in(cm);
        const double oz = simulation.map_offset.z.force_numerical_value_in(cm);
        config.boundaries = types::MappingBounds{
            ox * x_extent[cm], (ox + static_cast<double>(shape[0]) * res) * x_extent[cm],
            oy * y_extent[cm], (oy + static_cast<double>(shape[1]) * res) * y_extent[cm],
            oz * z_extent[cm], (oz + static_cast<double>(shape[2]) * res) * z_extent[cm],
        };
    }
    return config;
}

types::MapConfig SimulationRunFactoryImpl::outputMapConfig(const types::MissionConfigData& mission) {
    types::MapConfig config{};
    config.boundaries = mission.mission_bounds;
    // Output map anchored at the mission-bounds minimum corner.
    config.offset = Position3D{
        mission.mission_bounds.min_x.force_numerical_value_in(cm) * x_extent[cm],
        mission.mission_bounds.min_y.force_numerical_value_in(cm) * y_extent[cm],
        mission.mission_bounds.min_height.force_numerical_value_in(cm) * z_extent[cm],
    };
    config.resolution = mission.gps_resolution;
    return config;
}

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    // Hidden (ground-truth) map loaded from the simulation NPY file.
    std::shared_ptr<NpyArray> hidden_array = loadNpyArray(simulation.map_filename);
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_array, hiddenMapConfig(*hidden_array, simulation));

    // Output map: a dense grid covering the mission bounds, filled Unmapped.
    // Invalid mission bounds yield an empty array; MissionControl reports it.
    auto output_map = std::make_unique<Map3DImpl>(std::make_shared<NpyArray>(), outputMapConfig(mission));

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps, drone);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        mission,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);

    // Per-run output directory (path scheme chosen by SimulationManager).
    std::error_code ec;
    std::filesystem::create_directories(output_path, ec);
    const std::filesystem::path output_map_file = output_path / "output_map.npy";

    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file);
}

} // namespace drone_mapper
