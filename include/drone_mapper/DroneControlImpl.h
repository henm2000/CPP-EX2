#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMutableMap3D.h>

#include <optional>

namespace drone_mapper {

class DroneControlImpl final : public IDroneControl {
public:
    DroneControlImpl(types::DroneConfigData drone,
                     types::MissionConfigData mission,
                     ILidar& lidar,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    [[nodiscard]] types::DroneStepResult step() override;
    [[nodiscard]] types::DroneState state() const override;

private:
    // Applies one algorithm-issued movement command to the actuator.
    [[nodiscard]] types::MovementResult applyMovement(const types::MovementCommand& command);
    // Marks the drone's body footprint Empty in the output map. The drone
    // physically occupies this region (it got here without colliding), so it is
    // provably free space; this bootstraps the algorithm's body-fit checks,
    // which the sparse lidar cones alone cannot satisfy near the drone.
    void markBodyEmpty(const Position3D& center);

    types::DroneConfigData drone_;
    // Retained from the prescribed constructor; the mapping algorithm owns its
    // own copy of the mission config, so DroneControl itself does not read it.
    [[maybe_unused]] types::MissionConfigData mission_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
    // The most recent scan, fed back to the algorithm on the next step.
    std::optional<types::LidarScanResult> last_scan_;
};

} // namespace drone_mapper
