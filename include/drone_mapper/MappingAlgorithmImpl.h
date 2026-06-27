#pragma once

#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/VoxelKey.h>

#include <array>
#include <optional>
#include <queue>
#include <unordered_set>
#include <vector>

namespace drone_mapper {

// Reactive BFS frontier explorer (port of HW1 Drone). Each nextStep() emits a
// single MappingStepCommand: it scans the six axis directions at every newly
// entered cell, expands the BFS frontier with reachable Empty + body-fitting
// neighbours, then walks the BFS path to the next frontier cell one move
// command at a time. The shared output map (populated by ScanResultToVoxels via
// DroneControl) is the drone's only knowledge source; only Empty voxels are
// passable. When no frontier cell remains reachable it reports Finished.
class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    // Sub-voxel sampling fraction for the body-fit check (small enough not to
    // skip a single voxel along any axis).
    static constexpr double kSubStep = 0.5;
    // Floating-point inset so a body face sitting exactly on a voxel boundary
    // does not round into the neighbour voxel.
    static constexpr double kEdgeInset = 1e-6;
    // A per-axis cell delta below this fraction of a voxel is treated as zero.
    static constexpr double kMoveEpsFraction = 0.1;
    // Six axis-neighbour offsets in (dx, dy, dz) voxel multiples.
    static constexpr std::array<std::array<int, 3>, 6> kAxisDeltas{{
        {{1, 0, 0}}, {{-1, 0, 0}},
        {{0, 1, 0}}, {{0, -1, 0}},
        {{0, 0, 1}}, {{0, 0, -1}},
    }};

    [[nodiscard]] static double normaliseDeg(double deg_value);
    [[nodiscard]] double resCm() const;
    [[nodiscard]] VoxelKey toKey(const Position3D& pos) const;
    [[nodiscard]] Position3D toCentre(const VoxelKey& key) const;

    // Empty in the output map AND the drone body fits entirely in Empty space.
    [[nodiscard]] bool isNavigable(const Position3D& pos) const;
    [[nodiscard]] bool canFitAt(const Position3D& pos) const;

    void queueScansForCurrentCell();
    void expandFrontier(const Position3D& pos);
    [[nodiscard]] std::optional<std::vector<Position3D>> popReachableTarget(const Position3D& from);
    [[nodiscard]] std::vector<Position3D> bfsPath(const Position3D& from, const VoxelKey& target_key) const;
    // Fills pending_moves_ with the rotate/advance/elevate commands that take the
    // drone from its current state to the given adjacent cell centre.
    void buildMovesToCell(const types::DroneState& state, const Position3D& target_centre);
    // Begins moving along path_ from path_index_; returns the first move command
    // toward the next distinct cell, or nullopt when the path is exhausted.
    [[nodiscard]] std::optional<types::MovementCommand> beginNextCellMove(const types::DroneState& state,
                                                                          const VoxelKey& current_key);

    bool started_ = false;
    bool finished_ = false;

    std::queue<Orientation> pending_scans_;
    std::queue<types::MovementCommand> pending_moves_;

    std::vector<Position3D> path_;
    std::size_t path_index_ = 0;

    std::queue<VoxelKey> frontier_;
    std::unordered_set<VoxelKey, VoxelKeyHash> in_frontier_;
    std::unordered_set<VoxelKey, VoxelKeyHash> visited_;
    std::unordered_set<VoxelKey, VoxelKeyHash> scanned_;
};

} // namespace drone_mapper
