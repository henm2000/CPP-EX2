#include <drone_mapper/MockMovement.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps, types::DroneConfigData drone_config)
    : gps_(gps), drone_config_(std::move(drone_config)) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const double max_deg = drone_config_.max_rotate.force_numerical_value_in(deg);
    // Clamp the requested magnitude to the per-command maximum, then sign it by
    // the requested direction (Left = positive, Right = negative).
    double magnitude = std::abs(angle.force_numerical_value_in(deg));
    if (max_deg > 0.0) {
        magnitude = std::min(magnitude, max_deg);
    }
    const double signed_deg =
        (direction == types::RotationDirection::Left) ? magnitude : -magnitude;

    const Orientation current = gps_.heading();
    double new_deg = current.horizontal.force_numerical_value_in(deg) + signed_deg;
    new_deg = std::fmod(new_deg, 360.0);
    if (new_deg < 0.0) {
        new_deg += 360.0;
    }
    gps_.setHeading(Orientation{new_deg * horizontal_angle[deg], current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const double max_cm = drone_config_.max_advance.force_numerical_value_in(cm);
    double dist_cm = distance.force_numerical_value_in(cm);
    if (max_cm > 0.0) {
        dist_cm = std::clamp(dist_cm, -max_cm, max_cm);
    }

    // The drone advances along its current horizontal heading (no pitch).
    const Orientation heading = gps_.heading();
    const double dx = si::cos(heading.horizontal).force_numerical_value_in(mp::one);
    const double dy = si::sin(heading.horizontal).force_numerical_value_in(mp::one);

    const Position3D cur = gps_.position();
    gps_.setPosition(Position3D{
        cur.x + dist_cm * dx * x_extent[cm],
        cur.y + dist_cm * dy * y_extent[cm],
        cur.z,
    });
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const double max_cm = drone_config_.max_elevate.force_numerical_value_in(cm);
    double dist_cm = distance.force_numerical_value_in(cm);
    if (max_cm > 0.0) {
        dist_cm = std::clamp(dist_cm, -max_cm, max_cm);
    }

    const Position3D cur = gps_.position();
    gps_.setPosition(Position3D{
        cur.x,
        cur.y,
        cur.z + dist_cm * z_extent[cm],
    });
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
