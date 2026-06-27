#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/types/DroneTypes.h>

namespace drone_mapper {

// rotate() updates the heading only — the drone is a perfect sphere
// (diameter = DroneConfigData::dimensions), so an in-place rotation can never
// collide.  advance() and elevate() walk the proposed path in 0.2*resolution
// increments, sampling the full body cross-section at every step against the
// hidden world map.  On collision they return {false, message} and leave the
// position unchanged.
class MockMovement final : public IDroneMovement {
public:
    // gps       — mutable: position/heading are updated after each command.
    // world_map — read-only ground truth used for collision detection; the
    //             sampling step is 0.2 * world_map.getMapConfig().resolution.
    MockMovement(MockGPS& gps, const IMap3D& world_map, types::DroneConfigData drone);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    static constexpr double kSubStep             = 0.2;   // sub-voxel sampling step as a fraction of resolution
    static constexpr double kEdgeInset           = 1e-6;  // shrink body to avoid fp false-collisions on voxel faces
    static constexpr double kLoopSlack           = 1e-9;  // tolerance so the body-sweep loop keeps its last sample
    static constexpr double kShortMoveHalfFactor = 0.5;   // skip path check if move < half a sub-step
    static constexpr double kHorizontalDirEps    = 0.1;   // |dx|+|dy| above this counts as horizontal

    [[nodiscard]] bool isOccupied(const Position3D& pos) const;

    // True if the drone body overlaps any occupied voxel at any sample along
    // a path of [distance_cm] cm starting at [start] in direction (dx, dy, dz),
    // or at the endpoint.
    [[nodiscard]] bool pathBlocked(const Position3D& start,
                                   double dx, double dy, double dz,
                                   double distance_cm) const;

    MockGPS& gps_;
    const IMap3D& world_map_;
    types::DroneConfigData drone_;
};

} // namespace drone_mapper
