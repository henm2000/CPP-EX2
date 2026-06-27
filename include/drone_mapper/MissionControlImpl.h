#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/IMutableMap3D.h>

#include <filesystem>
#include <string>

namespace drone_mapper {

// Runs one mission: steps the drone up to max_steps, validates each move against
// the ground-truth hidden map (the drone is a sphere, so a body overlap with an
// Occupied voxel is a DRONE_HITS_OBSTACLE collision), logs every error the
// moment it occurs to the per-run error log, and saves the output map.
class MissionControlImpl final : public IMissionControl {
public:
    MissionControlImpl(types::MissionConfigData mission,
                       types::DroneConfigData drone,
                       const IMap3D& hidden_map,
                       IMutableMap3D& output_map,
                       IDroneControl& drone_control,
                       std::filesystem::path output_map_file);

    [[nodiscard]] types::MissionRunResult runMission() override;

private:
    // Sub-voxel sampling fraction for the swept body collision check.
    static constexpr double kSubStep = 0.5;

    [[nodiscard]] bool missionBoundsValid() const;
    // True iff the drone body, swept from `from` to `to`, overlaps any Occupied
    // voxel in the hidden map.
    [[nodiscard]] bool bodyCollides(const Position3D& from, const Position3D& to) const;
    [[nodiscard]] bool bodyOverlapsObstacle(const Position3D& centre, double radius_cm, double step_cm) const;
    // Appends a line to the per-run error log immediately (created on first use).
    void logError(const std::string& code, const std::string& message) const;

    types::MissionConfigData mission_;
    types::DroneConfigData drone_;
    const IMap3D& hidden_map_;
    IMutableMap3D& output_map_;
    IDroneControl& drone_control_;
    std::filesystem::path output_map_file_;
};

} // namespace drone_mapper
