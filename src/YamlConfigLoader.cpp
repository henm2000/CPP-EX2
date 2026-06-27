#include <drone_mapper/YamlConfigLoader.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace drone_mapper {

double YamlConfigLoader::readNum(const YAML::Node& node, double fallback) {
    try {
        return (node && !node.IsNull()) ? node.as<double>() : fallback;
    } catch (...) {
        return fallback;
    }
}

std::string YamlConfigLoader::readStr(const YAML::Node& node, const std::string& fallback) {
    try {
        return (node && !node.IsNull()) ? node.as<std::string>() : fallback;
    } catch (...) {
        return fallback;
    }
}

types::MappingBounds YamlConfigLoader::readBounds(const YAML::Node& boundaries) {
    types::MappingBounds bounds{};
    if (const YAML::Node xb = boundaries["x_boundary"]) {
        bounds.min_x = readNum(xb["min_cm"], 0.0) * x_extent[cm];
        bounds.max_x = readNum(xb["max_cm"], 0.0) * x_extent[cm];
    }
    if (const YAML::Node yb = boundaries["y_boundary"]) {
        bounds.min_y = readNum(yb["min_cm"], 0.0) * y_extent[cm];
        bounds.max_y = readNum(yb["max_cm"], 0.0) * y_extent[cm];
    }
    if (const YAML::Node hb = boundaries["height_boundary"]) {
        bounds.min_height = readNum(hb["min_cm"], 0.0) * z_extent[cm];
        bounds.max_height = readNum(hb["max_cm"], 0.0) * z_extent[cm];
    }
    return bounds;
}

types::SimulationConfigData YamlConfigLoader::loadSimulationConfig(const std::filesystem::path& path) {
    types::SimulationConfigData config{};
    config.map_resolution = 10.0 * cm;
    try {
        const YAML::Node root = YAML::LoadFile(path.string());
        const YAML::Node n = root["simulation_config"] ? root["simulation_config"] : root;

        config.map_filename = readStr(n["map_filename"], "");
        config.map_resolution = readNum(n["map_resolution_cm"], 10.0) * cm;
        config.initial_angle = readNum(n["initial_angle_deg"], 0.0) * horizontal_angle[deg];

        if (const YAML::Node off = n["map_axes_offset"]) {
            config.map_offset = Position3D{
                readNum(off["x_offset"], 0.0) * x_extent[cm],
                readNum(off["y_offset"], 0.0) * y_extent[cm],
                readNum(off["height_offset"], 0.0) * z_extent[cm],
            };
        }
        if (const YAML::Node ip = n["initial_drone_position"]) {
            config.initial_drone_position = Position3D{
                readNum(ip["x_cm"], 0.0) * x_extent[cm],
                readNum(ip["y_cm"], 0.0) * y_extent[cm],
                readNum(ip["height_cm"], 0.0) * z_extent[cm],
            };
        }
    } catch (...) {
        // Leave defaults in place; an empty map_filename surfaces as a -1 run.
    }
    return config;
}

types::MissionConfigData YamlConfigLoader::loadMissionConfig(const std::filesystem::path& path) {
    types::MissionConfigData config{};
    config.max_steps = 2400;
    config.gps_resolution = 10.0 * cm;
    config.output_mapping_resolution_factor = 1.0;
    try {
        const YAML::Node root = YAML::LoadFile(path.string());
        const YAML::Node n = root["mission_config"] ? root["mission_config"] : root;

        config.max_steps = static_cast<std::size_t>(readNum(n["max_steps"], 2400.0));
        config.gps_resolution = readNum(n["gps_resolution_cm"], 10.0) * cm;
        config.output_mapping_resolution_factor =
            readNum(n["output_mapping_resolution_factor"], 1.0);
        if (const YAML::Node b = n["boundaries"]) {
            config.mission_bounds = readBounds(b);
        }
    } catch (...) {
    }
    return config;
}

types::DroneConfigData YamlConfigLoader::loadDroneConfig(const std::filesystem::path& path) {
    types::DroneConfigData config{};
    // Defaults mirror the assignment example (diameter 30 -> radius 15).
    config.radius = 15.0 * cm;
    config.max_rotate = 45.0 * horizontal_angle[deg];
    config.max_advance = 50.0 * cm;
    config.max_elevate = 40.0 * cm;
    try {
        const YAML::Node root = YAML::LoadFile(path.string());
        const YAML::Node n = root["drone_config"] ? root["drone_config"] : root;

        // dimensions_cm is the sphere diameter the drone fits through; radius = half.
        config.radius = (readNum(n["dimensions_cm"], 30.0) / 2.0) * cm;
        config.max_rotate = readNum(n["max_rotate_deg"], 45.0) * horizontal_angle[deg];
        config.max_advance = readNum(n["max_advance_cm"], 50.0) * cm;
        config.max_elevate = readNum(n["max_elevate_cm"], 40.0) * cm;
    } catch (...) {
    }
    return config;
}

types::LidarConfigData YamlConfigLoader::loadLidarConfig(const std::filesystem::path& path) {
    types::LidarConfigData config{};
    config.z_min = 20.0 * cm;
    config.z_max = 120.0 * cm;
    config.d = 2.5 * cm;
    config.fov_circles = 5;
    try {
        const YAML::Node root = YAML::LoadFile(path.string());
        const YAML::Node n = root["lidar_config"] ? root["lidar_config"] : root;

        config.z_min = readNum(n["z_min_cm"], 20.0) * cm;
        config.z_max = readNum(n["z_max_cm"], 120.0) * cm;
        config.d = readNum(n["d_cm"], 2.5) * cm;
        config.fov_circles = static_cast<std::size_t>(readNum(n["fov_circles"], 5.0));
    } catch (...) {
    }
    return config;
}

types::SimulationCompositionData YamlConfigLoader::loadComposition(const std::filesystem::path& path) {
    types::SimulationCompositionData composition{};
    composition.composition_file = path;

    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to load composition '" + path.string() + "': " + e.what());
    }

    const std::filesystem::path base = path.has_parent_path() ? path.parent_path() : std::filesystem::path{"."};
    const auto resolve = [&base](const std::string& rel) -> std::filesystem::path {
        const std::filesystem::path p{rel};
        return p.is_absolute() ? p : (base / p);
    };

    const YAML::Node comp = root["simulation_compositions"] ? root["simulation_compositions"] : root;

    if (const YAML::Node sims = comp["simulations"]) {
        for (const YAML::Node& sim_entry : sims) {
            const std::string sim_path = readStr(sim_entry["simulation_config"], "");
            types::SimulationConfigData sim = loadSimulationConfig(resolve(sim_path));
            if (!sim.map_filename.empty() && sim.map_filename.is_relative()) {
                sim.map_filename = resolve(sim.map_filename.string());
            }

            std::vector<types::MissionConfigData> missions;
            if (const YAML::Node mission_paths = sim_entry["mission_configs"]) {
                for (const YAML::Node& m : mission_paths) {
                    missions.push_back(loadMissionConfig(resolve(readStr(m, ""))));
                }
            }
            composition.simulation_mission_groups.emplace_back(std::move(sim), std::move(missions));
        }
    }

    if (const YAML::Node drones = comp["drone_configs"]) {
        for (const YAML::Node& d : drones) {
            composition.drones.push_back(loadDroneConfig(resolve(readStr(d, ""))));
        }
    }
    if (const YAML::Node lidars = comp["lidar_configs"]) {
        for (const YAML::Node& l : lidars) {
            composition.lidars.push_back(loadLidarConfig(resolve(readStr(l, ""))));
        }
    }

    return composition;
}

YamlConfigLoader::ComparisonConfig
YamlConfigLoader::loadComparisonConfig(const std::filesystem::path& path) {
    ComparisonConfig config{};

    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to load comparison config '" + path.string() + "': " + e.what());
    }

    const YAML::Node cc = root["comparison_config"] ? root["comparison_config"] : root;

    const auto readEntry = [](const YAML::Node& section) -> MapComparisonEntry {
        MapComparisonEntry entry{};
        if (!section) {
            return entry;
        }
        entry.provided = true;
        if (const YAML::Node res = section["map_res_cm"]) {
            entry.has_resolution = true;
            entry.resolution = readNum(res, 0.0) * cm;
        }
        if (const YAML::Node off = section["map_offset"]) {
            entry.has_offset = true;
            entry.offset = Position3D{
                readNum(off["x_offset"], 0.0) * x_extent[cm],
                readNum(off["y_offset"], 0.0) * y_extent[cm],
                readNum(off["height_offset"], 0.0) * z_extent[cm],
            };
        }
        if (const YAML::Node bounds = section["map_boundaries"]) {
            entry.has_boundaries = true;
            entry.boundaries = readBounds(bounds);
        }
        return entry;
    };

    config.original = readEntry(cc["original"]);
    config.target = readEntry(cc["target"]);
    return config;
}

} // namespace drone_mapper
