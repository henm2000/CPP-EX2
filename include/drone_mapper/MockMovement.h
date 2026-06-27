#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {

// Simulation actuator: applies Rotate/Advance/Elevate to the MockGPS, clamping
// each command to the drone's configured per-command maximum. It does NOT do
// collision detection (the drone is a perfect sphere so rotation is always
// safe, and obstacle hits are validated against the hidden map by
// MissionControl, which is the component that owns the ground-truth map).
class MockMovement final : public IDroneMovement {
public:
    // Changed (extended): also takes the drone config so it can enforce the
    // per-command movement limits, matching HW1's MockDroneMovementDriver.
    MockMovement(MockGPS& gps, types::DroneConfigData drone_config);

    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    types::MovementResult advance(PhysicalLength distance) override;
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    MockGPS& gps_;
    types::DroneConfigData drone_config_;
};

} // namespace drone_mapper
