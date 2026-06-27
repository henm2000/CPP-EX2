#include <drone_mapper/SimulationManager.h>

#include <chrono>
#include <ctime>
#include <exception>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

std::string SimulationManager::generatedAtUtc() {
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &now);
#else
    gmtime_r(&now, &tm_utc);
#endif
    char buffer[32] = {};
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
    return std::string{buffer};
}

types::SimulationResult SimulationManager::makeErrorResult(
    const types::SimulationConfigData& simulation,
    const types::MissionConfigData& mission,
    const std::string& code,
    const std::string& message) {
    types::SimulationResult result{};
    result.simulation_config = simulation;
    result.mission_config = mission;
    result.resolution_request_status = types::ResolutionRequestStatus::Ignored;
    result.mission_results = {
        types::MissionRunResult{types::MissionRunStatus::Error, 0, {types::ErrorRef{code, message}}}};
    result.mission_score = -1.0;
    return result;
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;

    std::size_t group_index = 0;
    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        const std::string sim_stem =
            "sim" + std::to_string(group_index) + "_" + simulation.map_filename.stem().string();

        std::size_t mission_index = 0;
        for (const types::MissionConfigData& mission : missions) {
            const std::string mission_dir = "mission" + std::to_string(mission_index);

            std::size_t drone_index = 0;
            for (const types::DroneConfigData& drone : composition.drones) {
                std::size_t lidar_index = 0;
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    const std::string leaf =
                        "drone" + std::to_string(drone_index) + "__lidar" + std::to_string(lidar_index);
                    const std::filesystem::path run_dir =
                        output_path / "output_results" / sim_stem / mission_dir / leaf;

                    // A failure to create or run a single simulation must not
                    // abort the whole composition: it becomes a -1 result.
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation, mission, drone, lidar, run_dir);
                        runs.push_back(run->run());
                    } catch (const std::exception& e) {
                        runs.push_back(makeErrorResult(simulation, mission, "RUN_FAILED", e.what()));
                    }
                    ++lidar_index;
                }
                ++drone_index;
            }
            ++mission_index;
        }
        ++group_index;
    }

    types::SimulationManagerReport report{};
    report.generated_at_utc = generatedAtUtc();
    report.metric = "output_map_accuracy";
    report.score_range = std::make_tuple(0.0, 100.0);
    report.error_score = -1;
    report.runs = std::move(runs);
    return report;
}

} // namespace drone_mapper
