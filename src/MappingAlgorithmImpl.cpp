#include <drone_mapper/MappingAlgorithmImpl.h>

#include <drone_mapper/IMap3D.h>

#include <mp-units/systems/si/math.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_map>

namespace drone_mapper {

double MappingAlgorithmImpl::normaliseDeg(double deg_value) {
    while (deg_value > 180.0) deg_value -= 360.0;
    while (deg_value < -180.0) deg_value += 360.0;
    return deg_value;
}

double MappingAlgorithmImpl::resCm() const {
    double res = output_map_.getMapConfig().resolution.force_numerical_value_in(cm);
    if (res <= 0.0) {
        res = mission_config_.gps_resolution.force_numerical_value_in(cm);
    }
    return (res > 0.0) ? res : 1.0;
}

VoxelKey MappingAlgorithmImpl::toKey(const Position3D& pos) const {
    const double res = resCm();
    const types::MapConfig cfg = output_map_.getMapConfig();
    const double ox = cfg.offset.x.force_numerical_value_in(cm);
    const double oy = cfg.offset.y.force_numerical_value_in(cm);
    const double oz = cfg.offset.z.force_numerical_value_in(cm);
    return {
        static_cast<int>(std::floor((pos.x.force_numerical_value_in(cm) - ox) / res)),
        static_cast<int>(std::floor((pos.y.force_numerical_value_in(cm) - oy) / res)),
        static_cast<int>(std::floor((pos.z.force_numerical_value_in(cm) - oz) / res)),
    };
}

Position3D MappingAlgorithmImpl::toCentre(const VoxelKey& key) const {
    const double res = resCm();
    const types::MapConfig cfg = output_map_.getMapConfig();
    const double ox = cfg.offset.x.force_numerical_value_in(cm);
    const double oy = cfg.offset.y.force_numerical_value_in(cm);
    const double oz = cfg.offset.z.force_numerical_value_in(cm);
    return {
        (ox + (std::get<0>(key) + 0.5) * res) * x_extent[cm],
        (oy + (std::get<1>(key) + 0.5) * res) * y_extent[cm],
        (oz + (std::get<2>(key) + 0.5) * res) * z_extent[cm],
    };
}

bool MappingAlgorithmImpl::canFitAt(const Position3D& pos) const {
    const double res = resCm();
    double step = kSubStep * res;
    if (step <= 0.0) {
        step = res;
    }
    const double radius =
        std::max(0.0, drone_config_.radius.force_numerical_value_in(cm) - kEdgeInset);
    const double slack = 1e-9;

    for (double wx = -radius; wx <= radius + slack; wx += step) {
        for (double wy = -radius; wy <= radius + slack; wy += step) {
            for (double wz = -radius; wz <= radius + slack; wz += step) {
                const Position3D sample{
                    pos.x + wx * x_extent[cm],
                    pos.y + wy * y_extent[cm],
                    pos.z + wz * z_extent[cm],
                };
                if (output_map_.atVoxel(sample) != types::VoxelOccupancy::Empty) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool MappingAlgorithmImpl::isNavigable(const Position3D& pos) const {
    return output_map_.atVoxel(pos) == types::VoxelOccupancy::Empty && canFitAt(pos);
}

void MappingAlgorithmImpl::queueScansForCurrentCell() {
    // Six relative scan directions: the four cardinal headings plus angled
    // up/down probes, matching HW1's kScanDirs.
    constexpr double kProbeAltitudeDeg = 60.0;
    const std::array<Orientation, 6> dirs{{
        {0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        {90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        {180.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        {270.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
        {0.0 * horizontal_angle[deg], kProbeAltitudeDeg * altitude_angle[deg]},
        {0.0 * horizontal_angle[deg], -kProbeAltitudeDeg * altitude_angle[deg]},
    }};
    for (const Orientation& o : dirs) {
        pending_scans_.push(o);
    }
}

void MappingAlgorithmImpl::expandFrontier(const Position3D& pos) {
    const double res = resCm();
    const Position3D base = toCentre(toKey(pos));
    for (const auto& d : kAxisDeltas) {
        const Position3D nb{
            base.x + d[0] * res * x_extent[cm],
            base.y + d[1] * res * y_extent[cm],
            base.z + d[2] * res * z_extent[cm],
        };
        const VoxelKey key = toKey(nb);
        if (visited_.count(key) || in_frontier_.count(key)) {
            continue;
        }
        if (!isNavigable(nb)) {
            continue;
        }
        in_frontier_.insert(key);
        frontier_.push(key);
    }
}

std::optional<std::vector<Position3D>>
MappingAlgorithmImpl::popReachableTarget(const Position3D& from) {
    while (!frontier_.empty()) {
        const VoxelKey candidate = frontier_.front();
        frontier_.pop();
        in_frontier_.erase(candidate);
        if (visited_.count(candidate)) {
            continue;
        }
        std::vector<Position3D> path = bfsPath(from, candidate);
        if (!path.empty()) {
            return path;
        }
    }
    return std::nullopt;
}

std::vector<Position3D>
MappingAlgorithmImpl::bfsPath(const Position3D& from, const VoxelKey& target_key) const {
    const double res = resCm();
    std::unordered_map<VoxelKey, VoxelKey, VoxelKeyHash> parent;
    std::queue<VoxelKey> q;
    const VoxelKey start = toKey(from);

    q.push(start);
    parent[start] = start;

    while (!q.empty()) {
        const VoxelKey cur = q.front();
        q.pop();
        if (cur == target_key) {
            break;
        }
        const Position3D cur_centre = toCentre(cur);
        for (const auto& d : kAxisDeltas) {
            const Position3D nxt{
                cur_centre.x + d[0] * res * x_extent[cm],
                cur_centre.y + d[1] * res * y_extent[cm],
                cur_centre.z + d[2] * res * z_extent[cm],
            };
            const VoxelKey nxt_key = toKey(nxt);
            if (parent.count(nxt_key)) {
                continue;
            }
            // The target cell is itself a frontier cell (Empty + fits); every
            // intermediate cell must be navigable too.
            if (nxt_key == target_key || isNavigable(nxt)) {
                parent[nxt_key] = cur;
                q.push(nxt_key);
            }
        }
    }

    if (!parent.count(target_key)) {
        return {};
    }

    std::vector<VoxelKey> rev;
    for (VoxelKey k = target_key; k != start; k = parent.at(k)) {
        rev.push_back(k);
    }
    std::reverse(rev.begin(), rev.end());

    std::vector<Position3D> path;
    path.reserve(rev.size());
    for (const VoxelKey& k : rev) {
        path.push_back(toCentre(k));
    }
    return path;
}

void MappingAlgorithmImpl::buildMovesToCell(const types::DroneState& state,
                                            const Position3D& target_centre) {
    const double res = resCm();
    const double residual = kMoveEpsFraction * res;

    const double dx = target_centre.x.force_numerical_value_in(cm) -
                      state.position.x.force_numerical_value_in(cm);
    const double dy = target_centre.y.force_numerical_value_in(cm) -
                      state.position.y.force_numerical_value_in(cm);
    const double dz = target_centre.z.force_numerical_value_in(cm) -
                      state.position.z.force_numerical_value_in(cm);

    // 1. Vertical move first (elevate, signed).
    if (std::abs(dz) > residual) {
        const double max_ele = drone_config_.max_elevate.force_numerical_value_in(cm);
        const double sign = (dz >= 0.0) ? 1.0 : -1.0;
        double remaining = std::abs(dz);
        while (remaining > residual) {
            const double chunk = (max_ele > 0.0) ? std::min(remaining, max_ele) : remaining;
            types::MovementCommand cmd;
            cmd.type = types::MovementCommandType::Elevate;
            cmd.distance = (sign * chunk) * cm;
            pending_moves_.push(cmd);
            remaining -= chunk;
            if (max_ele <= 0.0) {
                break;
            }
        }
    }

    // 2. Horizontal move: rotate to face the target, then advance.
    const double horiz = std::hypot(dx, dy);
    if (horiz > residual) {
        const double target_bearing = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
        const double current = state.heading.horizontal.force_numerical_value_in(deg);
        const double delta = normaliseDeg(target_bearing - current);
        const types::RotationDirection dir =
            (delta >= 0.0) ? types::RotationDirection::Left : types::RotationDirection::Right;
        const double max_rot = drone_config_.max_rotate.force_numerical_value_in(deg);
        const double rot_tol = 0.5;
        double remaining_rot = std::abs(delta);
        while (remaining_rot > rot_tol) {
            const double chunk = (max_rot > 0.0) ? std::min(remaining_rot, max_rot) : remaining_rot;
            types::MovementCommand cmd;
            cmd.type = types::MovementCommandType::Rotate;
            cmd.rotation = dir;
            cmd.angle = chunk * horizontal_angle[deg];
            pending_moves_.push(cmd);
            remaining_rot -= chunk;
            if (max_rot <= 0.0) {
                break;
            }
        }

        const double max_adv = drone_config_.max_advance.force_numerical_value_in(cm);
        double remaining_adv = horiz;
        while (remaining_adv > residual) {
            const double chunk = (max_adv > 0.0) ? std::min(remaining_adv, max_adv) : remaining_adv;
            types::MovementCommand cmd;
            cmd.type = types::MovementCommandType::Advance;
            cmd.distance = chunk * cm;
            pending_moves_.push(cmd);
            remaining_adv -= chunk;
            if (max_adv <= 0.0) {
                break;
            }
        }
    }
}

std::optional<types::MovementCommand>
MappingAlgorithmImpl::beginNextCellMove(const types::DroneState& state, const VoxelKey& current_key) {
    while (path_index_ < path_.size()) {
        const Position3D centre = path_[path_index_];
        ++path_index_;
        const VoxelKey key = toKey(centre);
        if (key == current_key) {
            continue;
        }
        buildMovesToCell(state, centre);
        visited_.insert(key);
        if (!pending_moves_.empty()) {
            const types::MovementCommand cmd = pending_moves_.front();
            pending_moves_.pop();
            return cmd;
        }
    }
    return std::nullopt;
}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                        const types::LidarScanResult* latest_scan) {
    (void)latest_scan; // observations already applied to output_map_ by DroneControl.

    if (finished_) {
        types::MappingStepCommand done;
        done.status = types::AlgorithmStatus::Finished;
        return done;
    }

    const Position3D cur = state.position;
    const VoxelKey cur_key = toKey(cur);

    if (!started_) {
        started_ = true;
        visited_.insert(cur_key);
    }

    // 1. Drain any pending movement commands toward the current path cell.
    if (!pending_moves_.empty()) {
        types::MappingStepCommand cmd;
        cmd.movement = pending_moves_.front();
        pending_moves_.pop();
        return cmd;
    }

    // 2. On arrival at a not-yet-scanned cell, queue its six scans.
    if (scanned_.find(cur_key) == scanned_.end()) {
        queueScansForCurrentCell();
        scanned_.insert(cur_key);
    }

    // 3. Emit the next pending scan.
    if (!pending_scans_.empty()) {
        types::MappingStepCommand cmd;
        cmd.scan_orientation = pending_scans_.front();
        pending_scans_.pop();
        return cmd;
    }

    // 4. Scans done at this cell: expand the frontier with new reachable cells.
    expandFrontier(cur);

    // 5. Keep walking the current path if one is active.
    if (std::optional<types::MovementCommand> mv = beginNextCellMove(state, cur_key)) {
        types::MappingStepCommand cmd;
        cmd.movement = *mv;
        return cmd;
    }

    // 6. Path finished: choose the next reachable frontier target.
    path_.clear();
    path_index_ = 0;
    if (std::optional<std::vector<Position3D>> target = popReachableTarget(cur)) {
        path_ = std::move(*target);
        path_index_ = 0;
        if (std::optional<types::MovementCommand> mv = beginNextCellMove(state, cur_key)) {
            types::MappingStepCommand cmd;
            cmd.movement = *mv;
            return cmd;
        }
    }

    // 7. No reachable frontier remains: mapping is complete.
    finished_ = true;
    types::MappingStepCommand done;
    done.status = types::AlgorithmStatus::Finished;
    return done;
}

} // namespace drone_mapper
