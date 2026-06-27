#pragma once

#include <TinyNPY.h>

#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/Types.h>

#include <filesystem>
#include <memory>

namespace drone_mapper {

class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;

private:
    // Loads an NPY file into a shared array; throws std::runtime_error on failure
    // (caught by SimulationManager and turned into a -1 result for the group).
    [[nodiscard]] static std::shared_ptr<NpyArray> loadNpyArray(const std::filesystem::path& path);
    // Hidden (ground-truth) map config: offset + resolution from the simulation,
    // boundaries spanning the loaded array footprint.
    [[nodiscard]] static types::MapConfig hiddenMapConfig(const NpyArray& array,
                                                          const types::SimulationConfigData& simulation);
    // Output map config: boundaries = mission bounds, offset = bounds min corner,
    // resolution = the GPS resolution we map at.
    [[nodiscard]] static types::MapConfig outputMapConfig(const types::MissionConfigData& mission);
};

} // namespace drone_mapper
