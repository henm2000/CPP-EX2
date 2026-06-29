#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !drone_control_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    const types::MissionRunResult mission_result = mission_control_->runMission();

    // Resolution policy: we always map at the GPS/input resolution. A factor < 1
    // would request finer-than-GPS precision (impossible); a factor > 1 requests
    // a coarser output we do not honour; exactly 1 is the resolution we used.
    const double factor = mission_config_.output_mapping_resolution_factor;
    types::ResolutionRequestStatus resolution_status;
    if (factor < 1.0) {
        resolution_status = types::ResolutionRequestStatus::IgnoredTooSmall;
        // The PDF requires the "< 1" case to be logged immediately to the error log.
        std::error_code ec;
        const std::filesystem::path log_path = output_map_file_.parent_path() / "error_log.txt";
        std::filesystem::create_directories(log_path.parent_path(), ec);
        std::ofstream log(log_path, std::ios::app);
        if (log) {
            log << "RESOLUTION_TOO_SMALL: output_mapping_resolution_factor < 1; "
                   "using the default (GPS) resolution\n";
        }
    } else if (std::abs(factor - 1.0) < 1e-9) {
        resolution_status = types::ResolutionRequestStatus::Accepted;
    } else {
        resolution_status = types::ResolutionRequestStatus::Ignored;
    }

    // Score = accuracy of the produced map vs the ground-truth hidden map.
    double score = -1.0;
    if (mission_result.status != types::MissionRunStatus::Error) {
        try {
            const std::vector<IMap3D*> targets{static_cast<IMap3D*>(output_map_.get())};
            const std::vector<double> scores = MapsComparison::compare(*hidden_map_, targets);
            score = scores.empty() ? -1.0 : scores.front();
        } catch (const std::exception&) {
            score = -1.0;
        }
    }

    return types::SimulationResult{
        simulation_config_,
        mission_config_,
        resolution_status,
        {mission_result},
        output_map_file_,
        output_map_->getMapConfig(),
        score,
    };
}

} // namespace drone_mapper
