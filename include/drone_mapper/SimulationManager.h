#pragma once

#include <drone_mapper/ISimulation.h>
#include <drone_mapper/ISimulationRunFactory.h>

#include <filesystem>
#include <memory>
#include <string>

namespace drone_mapper {

class SimulationManager final : public ISimulation {
public:
    explicit SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory);

    // Changed: matches ISimulation's new SimulationManagerReport return type.
    [[nodiscard]] types::SimulationManagerReport run(const types::SimulationCompositionData& composition,
                                              const std::filesystem::path& output_path) override; // output - to save the output map for example

private:
    // Current UTC time as an ISO-8601 "YYYY-MM-DDThh:mm:ssZ" string.
    [[nodiscard]] static std::string generatedAtUtc();
    // A score -1 result used when a run cannot even be created/run (e.g. its map
    // file fails to load), so its whole group is filled with the error score.
    [[nodiscard]] static types::SimulationResult makeErrorResult(
        const types::SimulationConfigData& simulation,
        const types::MissionConfigData& mission,
        const std::string& code,
        const std::string& message);

    std::unique_ptr<ISimulationRunFactory> run_factory_;
};

} // namespace drone_mapper
