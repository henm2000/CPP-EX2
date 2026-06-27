#pragma once

#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Types.h>

#include <vector>

namespace drone_mapper {

class MapsComparison {
public:
    [[nodiscard]] static std::vector<double> compare(const IMap3D& origin,
                                                     const std::vector<IMap3D*> targets); //currently should work with at least 1 target

private:
    // Accuracy (0..100) of one target vs the origin: the percentage of voxel
    // centres (sampled at the origin's resolution over the overlap of both
    // maps' boundaries) where both maps report the same occupancy. Throws when
    // the resolutions differ (different-resolution comparison is a bonus we do
    // not implement).
    [[nodiscard]] static double compareOne(const IMap3D& origin, const IMap3D& target);
};

} // namespace drone_mapper
