#include <drone_mapper/MockMovement.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps, const IMap3D& world_map, types::DroneConfigData drone)
    : gps_(gps), world_map_(world_map), drone_(drone) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const double max_deg = drone_.max_rotate.force_numerical_value_in(deg);
    const double magnitude = angle.force_numerical_value_in(deg);
    const double signed_deg =
        (direction == types::RotationDirection::Left) ? magnitude : -magnitude;
    const double clamped = std::clamp(signed_deg, -max_deg, max_deg);

    const Orientation current = gps_.heading();
    double new_deg = current.horizontal.force_numerical_value_in(deg) + clamped;
    new_deg = std::fmod(new_deg, 360.0);
    if (new_deg < 0.0) new_deg += 360.0;
    gps_.setHeading(Orientation{new_deg * horizontal_angle[deg], current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const double max_cm  = drone_.max_advance.force_numerical_value_in(cm);
    const double dist_cm = std::clamp(distance.force_numerical_value_in(cm), -max_cm, max_cm);

    const HorizontalAngle heading = gps_.heading().horizontal;
    const double dx = si::cos(heading).force_numerical_value_in(mp::one);
    const double dy = si::sin(heading).force_numerical_value_in(mp::one);

    if (pathBlocked(gps_.position(), dx, dy, 0.0, dist_cm)) {
        return types::MovementResult{false, "collision detected during advance"};
    }

    const Position3D cur = gps_.position();
    gps_.setPosition(Position3D{
        cur.x + dist_cm * dx * x_extent[cm],
        cur.y + dist_cm * dy * y_extent[cm],
        cur.z,
    });
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const double max_cm  = drone_.max_elevate.force_numerical_value_in(cm);
    const double dist_cm = std::clamp(distance.force_numerical_value_in(cm), -max_cm, max_cm);

    if (pathBlocked(gps_.position(), 0.0, 0.0, 1.0, dist_cm)) {
        return types::MovementResult{false, "collision detected during elevate"};
    }

    const Position3D cur = gps_.position();
    gps_.setPosition(Position3D{
        cur.x,
        cur.y,
        cur.z + dist_cm * z_extent[cm],
    });
    return types::MovementResult{true, {}};
}

bool MockMovement::isOccupied(const Position3D& pos) const {
    return world_map_.atVoxel(pos) == types::VoxelOccupancy::Occupied;
}

bool MockMovement::pathBlocked(const Position3D& start,
                               double dx, double dy, double dz,
                               double distance_cm) const {
    const double step_cm =
        kSubStep * world_map_.getMapConfig().resolution.force_numerical_value_in(cm);
    // Collision sampling needs a valid resolution; without one no sane step
    // size exists (loader/factory guarantee a positive value).

    const double abs_dist = std::abs(distance_cm);
    const double sign     = (distance_cm >= 0.0) ? 1.0 : -1.0;

    if (abs_dist < step_cm * kShortMoveHalfFactor) return false;

    // kEdgeInset shrinks the body by ~1e-6 cm so a sample whose surface sits
    // exactly on a voxel face doesn't fp-round into the neighbour voxel and
    // trigger a spurious collision.  The drone is a sphere, so the half-extent
    // is the same on every axis.
    const double radius = drone_.dimensions.force_numerical_value_in(cm) / 2.0 - kEdgeInset;

    // For a horizontal advance (dz == 0):
    //   lateral perpendicular in XY = (-dy, dx)
    //   vertical extent              = Z ± radius
    // For a vertical elevate (dx == dy == 0):
    //   check full XY footprint      = ±radius in X and Y
    const bool horizontal = (std::abs(dx) + std::abs(dy)) > kHorizontalDirEps;

    auto bodyBlocked = [this, &horizontal, &dx, &dy, &radius, &step_cm](
                           const Position3D& centre) -> bool {
        if (horizontal) {
            const double px = -dy;
            const double py =  dx;
            for (double w = -radius; w <= radius + kLoopSlack; w += step_cm) {
                for (double h = -radius; h <= radius + kLoopSlack; h += step_cm) {
                    const Position3D p{
                        centre.x + w * px * x_extent[cm],
                        centre.y + w * py * y_extent[cm],
                        centre.z + h      * z_extent[cm],
                    };
                    if (isOccupied(p)) return true;
                }
            }
        } else {
            for (double wx = -radius; wx <= radius + kLoopSlack; wx += step_cm) {
                for (double wy = -radius; wy <= radius + kLoopSlack; wy += step_cm) {
                    const Position3D p{
                        centre.x + wx * x_extent[cm],
                        centre.y + wy * y_extent[cm],
                        centre.z,
                    };
                    if (isOccupied(p)) return true;
                }
            }
        }
        return false;
    };

    // March cross-section slices from the start-center forward. We deliberately
    // begin at the center (d = step_cm), not a body-radius behind it: the rear
    // hemisphere is inside the start sphere, which the previous (checked) move
    // already cleared. We continue `radius` past the endpoint so the front of
    // the sphere — which reaches `radius` cm beyond the final center — is tested
    // too; otherwise a wall just ahead of the endpoint, already inside the body,
    // would slip through unchecked.
    const double check_dist = abs_dist + radius;
    for (double d = step_cm; d <= check_dist + kLoopSlack; d += step_cm) {
        const Position3D sample{
            start.x + d * sign * dx * x_extent[cm],
            start.y + d * sign * dy * y_extent[cm],
            start.z + d * sign * dz * z_extent[cm],
        };
        if (bodyBlocked(sample)) return true;
    }
    return false;
}

} // namespace drone_mapper
