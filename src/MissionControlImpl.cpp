#include <drone_mapper/MissionControlImpl.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <system_error>
#include <utility>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

bool MissionControlImpl::missionBoundsValid() const {
    const types::MappingBounds& b = mission_.mission_bounds;
    return b.max_x > b.min_x && b.max_y > b.min_y && b.max_height > b.min_height;
}

bool MissionControlImpl::bodyOverlapsObstacle(const Position3D& centre,
                                              double radius_cm,
                                              double step_cm) const {
    const double slack = 1e-9;
    const double r = std::max(0.0, radius_cm);
    for (double wx = -r; wx <= r + slack; wx += step_cm) {
        for (double wy = -r; wy <= r + slack; wy += step_cm) {
            for (double wz = -r; wz <= r + slack; wz += step_cm) {
                const Position3D p{
                    centre.x + wx * x_extent[cm],
                    centre.y + wy * y_extent[cm],
                    centre.z + wz * z_extent[cm],
                };
                if (hidden_map_.atVoxel(p) == types::VoxelOccupancy::Occupied) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool MissionControlImpl::bodyCollides(const Position3D& from, const Position3D& to) const {
    double res = hidden_map_.getMapConfig().resolution.force_numerical_value_in(cm);
    if (res <= 0.0) {
        res = 1.0;
    }
    double step = kSubStep * res;
    if (step <= 0.0) {
        step = res;
    }
    const double radius = drone_.radius.force_numerical_value_in(cm);

    const double fx = from.x.force_numerical_value_in(cm);
    const double fy = from.y.force_numerical_value_in(cm);
    const double fz = from.z.force_numerical_value_in(cm);
    const double dx = to.x.force_numerical_value_in(cm) - fx;
    const double dy = to.y.force_numerical_value_in(cm) - fy;
    const double dz = to.z.force_numerical_value_in(cm) - fz;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    const int samples = (dist > step) ? static_cast<int>(std::ceil(dist / step)) : 1;
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        const Position3D point{
            (fx + t * dx) * x_extent[cm],
            (fy + t * dy) * y_extent[cm],
            (fz + t * dz) * z_extent[cm],
        };
        if (bodyOverlapsObstacle(point, radius, step)) {
            return true;
        }
    }
    return false;
}

void MissionControlImpl::logError(const std::string& code, const std::string& message) const {
    const std::filesystem::path log_path = output_map_file_.parent_path() / "error_log.txt";
    std::error_code ec;
    std::filesystem::create_directories(log_path.parent_path(), ec);
    // Append + immediate flush so errors are persisted the moment they occur.
    std::ofstream out(log_path, std::ios::app);
    if (out) {
        out << code << ": " << message << "\n";
    }
}

types::MissionRunResult MissionControlImpl::runMission() {
    // Invalid mission bounds are unrecoverable for this run: report immediately.
    if (!missionBoundsValid()) {
        logError("MISSION_BOUNDARY_INVALID", "mission boundaries are empty or inverted");
        return types::MissionRunResult{
            types::MissionRunStatus::Error,
            0,
            {types::ErrorRef{"MISSION_BOUNDARY_INVALID", "mission boundaries are empty or inverted"}},
        };
    }

    const double radius = drone_.radius.force_numerical_value_in(cm);
    Position3D prev = drone_control_.state().position;

    // The drone must not start embedded inside an obstacle.
    {
        double res = hidden_map_.getMapConfig().resolution.force_numerical_value_in(cm);
        if (res <= 0.0) {
            res = 1.0;
        }
        if (bodyOverlapsObstacle(prev, radius, kSubStep * res)) {
            logError("DRONE_HITS_OBSTACLE", "initial drone position is inside an obstacle");
            return types::MissionRunResult{
                types::MissionRunStatus::Error,
                0,
                {types::ErrorRef{"DRONE_HITS_OBSTACLE", "initial drone position is inside an obstacle"}},
            };
        }
    }

    std::size_t steps = 0;
    types::MissionRunStatus status = types::MissionRunStatus::MaxSteps;

    for (std::size_t i = 0; i < mission_.max_steps; ++i) {
        const types::DroneStepResult step_result = drone_control_.step();
        ++steps;

        if (step_result.status == types::DroneStepStatus::Error) {
            logError("DRONE_STEP_FAILED", step_result.message);
            // Save whatever was mapped before failing, then report the error.
            try {
                output_map_.save(output_map_file_);
            } catch (const std::exception&) {
            }
            return types::MissionRunResult{
                types::MissionRunStatus::Error,
                steps,
                {types::ErrorRef{"DRONE_STEP_FAILED", step_result.message}},
            };
        }

        const Position3D cur = drone_control_.state().position;
        if (bodyCollides(prev, cur)) {
            logError("DRONE_HITS_OBSTACLE", "drone body intersected an obstacle while moving");
            try {
                output_map_.save(output_map_file_);
            } catch (const std::exception&) {
            }
            return types::MissionRunResult{
                types::MissionRunStatus::Error,
                steps,
                {types::ErrorRef{"DRONE_HITS_OBSTACLE", "drone body intersected an obstacle while moving"}},
            };
        }
        prev = cur;

        if (step_result.status == types::DroneStepStatus::Completed) {
            status = types::MissionRunStatus::Completed;
            break;
        }
    }

    try {
        output_map_.save(output_map_file_);
    } catch (const std::exception& e) {
        logError("OUTPUT_SAVE_FAILED", e.what());
        return types::MissionRunResult{
            types::MissionRunStatus::Error,
            steps,
            {types::ErrorRef{"OUTPUT_SAVE_FAILED", e.what()}},
        };
    }

    return types::MissionRunResult{status, steps, {}};
}

} // namespace drone_mapper
