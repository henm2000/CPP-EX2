#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <algorithm>
#include <utility>

namespace drone_mapper {

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

void DroneControlImpl::markBodyEmpty(const Position3D& center) {
    const double res = output_map_.getMapConfig().resolution.force_numerical_value_in(cm);
    if (res <= 0.0) {
        return;
    }
    const double step = 0.5 * res;
    const double radius = std::max(0.0, drone_.radius.force_numerical_value_in(cm));
    const double slack = 1e-9;
    const double r2 = radius * radius;
    for (double wx = -radius; wx <= radius + slack; wx += step) {
        for (double wy = -radius; wy <= radius + slack; wy += step) {
            for (double wz = -radius; wz <= radius + slack; wz += step) {
                if (wx * wx + wy * wy + wz * wz > r2) {
                    continue; // only the spherical body is provably free space
                }
                const Position3D sample{
                    center.x + wx * x_extent[cm],
                    center.y + wy * y_extent[cm],
                    center.z + wz * z_extent[cm],
                };
                if (output_map_.isInBounds(sample) &&
                    output_map_.atVoxel(sample) != types::VoxelOccupancy::Occupied) {
                    output_map_.set(sample, types::VoxelOccupancy::Empty);
                }
            }
        }
    }
}

types::MovementResult DroneControlImpl::applyMovement(const types::MovementCommand& command) {
    switch (command.type) {
    case types::MovementCommandType::Rotate:
        return movement_.rotate(command.rotation, command.angle);
    case types::MovementCommandType::Advance:
        return movement_.advance(command.distance);
    case types::MovementCommandType::Elevate:
        return movement_.elevate(command.distance);
    case types::MovementCommandType::Hover:
        break;
    }
    return types::MovementResult{true, {}};
}

types::DroneStepResult DroneControlImpl::step() {
    // The drone's current footprint is provably free space; record it so the
    // algorithm's body-fit checks have a solid empty core to expand from.
    markBodyEmpty(gps_.position());

    const types::DroneState current = state();
    const types::LidarScanResult* latest = last_scan_ ? &*last_scan_ : nullptr;
    const types::MappingStepCommand command = mapping_algorithm_.nextStep(current, latest);

    if (command.status == types::AlgorithmStatus::Finished ||
        command.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return types::DroneStepResult{types::DroneStepStatus::Completed, "mapping finished"};
    }

    // The contract requires movement to be applied before any scan in the same step.
    if (command.movement.has_value()) {
        const types::MovementResult moved = applyMovement(*command.movement);
        if (!moved) {
            return types::DroneStepResult{types::DroneStepStatus::Error, moved.message};
        }
    }

    if (command.scan_orientation.has_value()) {
        types::LidarScanResult scan = lidar_.scan(*command.scan_orientation);
        ScanResultToVoxels::applyToMap(output_map_,
                                       gps_.position(),
                                       gps_.heading(),
                                       scan,
                                       lidar_.config());
        last_scan_ = std::move(scan);
    }

    ++step_index_;
    return types::DroneStepResult{types::DroneStepStatus::Continue, {}};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
