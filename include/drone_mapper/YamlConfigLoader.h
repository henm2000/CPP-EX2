#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>
#include <string>

// Forward declaration keeps the yaml-cpp dependency private to the .cpp while
// still letting the parsing helpers be declared as class members.
namespace YAML { class Node; }

namespace drone_mapper {

// Loads the YAML configuration files into the locked config structs. yaml-cpp is
// an implementation detail and is kept out of this header (the library links it
// PRIVATE). Missing keys fall back to sensible defaults; a value the parser
// cannot read leaves the corresponding default in place.
class YamlConfigLoader {
public:
    // Loads the top-level composition file. Referenced config paths (and the
    // simulation map_filename) are resolved relative to the composition file's
    // directory when relative. Throws std::runtime_error if the composition file
    // itself cannot be read.
    [[nodiscard]] static types::SimulationCompositionData loadComposition(const std::filesystem::path& path);

    [[nodiscard]] static types::SimulationConfigData loadSimulationConfig(const std::filesystem::path& path);
    [[nodiscard]] static types::MissionConfigData loadMissionConfig(const std::filesystem::path& path);
    [[nodiscard]] static types::DroneConfigData loadDroneConfig(const std::filesystem::path& path);
    [[nodiscard]] static types::LidarConfigData loadLidarConfig(const std::filesystem::path& path);

    // One map's section of a maps_comparison comparison_config file. Absent
    // fields are flagged so the caller can fall back to map-derived defaults.
    struct MapComparisonEntry {
        bool provided = false;
        bool has_resolution = false;
        PhysicalLength resolution{};
        bool has_offset = false;
        Position3D offset{};
        bool has_boundaries = false;
        types::MappingBounds boundaries{};
    };
    struct ComparisonConfig {
        MapComparisonEntry original;
        MapComparisonEntry target;
    };
    // Throws std::runtime_error if the file cannot be read.
    [[nodiscard]] static ComparisonConfig loadComparisonConfig(const std::filesystem::path& path);

private:
    // yaml-cpp leaf adapters (defined in the .cpp where yaml-cpp is included).
    [[nodiscard]] static double readNum(const YAML::Node& node, double fallback);
    [[nodiscard]] static std::string readStr(const YAML::Node& node, const std::string& fallback);
    [[nodiscard]] static types::MappingBounds readBounds(const YAML::Node& boundaries);
};

} // namespace drone_mapper
