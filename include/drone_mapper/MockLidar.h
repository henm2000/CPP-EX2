#pragma once

#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMap3D.h>

#include <cstddef>

namespace drone_mapper {

class MockLidar final : public ILidar {
public:
    MockLidar(types::LidarConfigData config, const IMap3D& map, const IGPS& gps);

    [[nodiscard]] types::LidarScanResult scan(Orientation scan_orientation) const override;

private:
    // Sub-voxel march step as a fraction of the map resolution. Small enough
    // that a ray cannot skip a single voxel along any axis-aligned or
    // near-axis-aligned direction. Shared convention with MockMovement's kSubStep.
    static constexpr double kStepFraction = 0.2;

    [[nodiscard]] PhysicalLength traceBeam(const Orientation& beam) const;

    // Number of beams on circle index N: 1, 4, 16, 64, ... (4^N).
    [[nodiscard]] static std::size_t beamsOnCircle(std::size_t circle_index);

    // Convert a planar offset at a reference distance into the angular delta
    // that points at it.
    [[nodiscard]] static HorizontalAngle horizontalDelta(PhysicalLength offset, PhysicalLength distance);
    [[nodiscard]] static AltitudeAngle altitudeDelta(PhysicalLength offset, PhysicalLength distance);

    types::LidarConfigData config_;
    const IMap3D& map_;
    const IGPS& gps_;
};

} // namespace drone_mapper