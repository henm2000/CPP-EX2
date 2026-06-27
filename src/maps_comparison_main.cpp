#include <TinyNPY.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/YamlConfigLoader.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ./maps_comparison <origin_map> <target_map> [comparison_config=<path>]
//   Prints a single accuracy number in [0, 100] to stdout (no other text).
//   On any error prints -1 to stdout and a message to stderr, and returns 1.
int main(int argc, char** argv) {
    namespace dm = drone_mapper;
    namespace fs = std::filesystem;

    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }

    const fs::path origin_path{argv[1]};
    const fs::path target_path{argv[2]};
    fs::path config_path;
    bool has_config = false;
    if (argc == 4) {
        const std::string arg = argv[3];
        const std::string prefix = "comparison_config=";
        config_path = (arg.rfind(prefix, 0) == 0) ? arg.substr(prefix.size()) : arg;
        has_config = true;
    }

    try {
        const auto loadArray = [](const fs::path& p) {
            auto array = std::make_shared<NpyArray>();
            const char* err = array->LoadNPY(p.string().c_str());
            if (err != nullptr) {
                throw std::runtime_error("failed to load '" + p.string() + "': " + err);
            }
            return array;
        };

        // Array footprint as world bounds: [offset, offset + shape * resolution).
        const auto footprint = [](const NpyArray& array, const dm::Position3D& off, dm::PhysicalLength res) {
            const auto& shape = array.Shape();
            const double r = res.force_numerical_value_in(dm::cm);
            const double ox = off.x.force_numerical_value_in(dm::cm);
            const double oy = off.y.force_numerical_value_in(dm::cm);
            const double oz = off.z.force_numerical_value_in(dm::cm);
            const double nx = shape.size() >= 3 ? static_cast<double>(shape[0]) : 0.0;
            const double ny = shape.size() >= 3 ? static_cast<double>(shape[1]) : 0.0;
            const double nz = shape.size() >= 3 ? static_cast<double>(shape[2]) : 0.0;
            return dm::types::MappingBounds{
                ox * dm::x_extent[dm::cm], (ox + nx * r) * dm::x_extent[dm::cm],
                oy * dm::y_extent[dm::cm], (oy + ny * r) * dm::y_extent[dm::cm],
                oz * dm::z_extent[dm::cm], (oz + nz * r) * dm::z_extent[dm::cm],
            };
        };

        dm::YamlConfigLoader::ComparisonConfig cfg{};
        if (has_config) {
            cfg = dm::YamlConfigLoader::loadComparisonConfig(config_path);
        }

        // No config => both maps share offset 0, resolution 1 cm, and their own
        // footprint as bounds (a cell-by-cell comparison).
        const auto buildConfig = [&footprint](const NpyArray& array,
                                              const dm::YamlConfigLoader::MapComparisonEntry& entry) {
            const dm::PhysicalLength res = entry.has_resolution ? entry.resolution : (1.0 * dm::cm);
            const dm::Position3D off = entry.has_offset ? entry.offset : dm::Position3D{};
            const dm::types::MappingBounds bounds =
                entry.has_boundaries ? entry.boundaries : footprint(array, off, res);
            return dm::types::MapConfig{bounds, off, res};
        };

        const std::shared_ptr<NpyArray> origin_array = loadArray(origin_path);
        const std::shared_ptr<NpyArray> target_array = loadArray(target_path);

        dm::Map3DImpl origin{origin_array, buildConfig(*origin_array, cfg.original)};
        dm::Map3DImpl target{target_array, buildConfig(*target_array, cfg.target)};

        const std::vector<dm::IMap3D*> targets{&target};
        const std::vector<double> scores = dm::MapsComparison::compare(origin, targets);
        std::cout << (scores.empty() ? -1.0 : scores.front()) << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cout << "-1\n";
        std::cerr << e.what() << "\n";
        return 1;
    }
}
