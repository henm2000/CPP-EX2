#include <drone_mapper/MapsComparison.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_mapper {

double MapsComparison::compareOne(const IMap3D& origin, const IMap3D& target) {
    const types::MapConfig oc = origin.getMapConfig();
    const types::MapConfig tc = target.getMapConfig();

    const double res = oc.resolution.force_numerical_value_in(cm);
    const double res_t = tc.resolution.force_numerical_value_in(cm);
    if (res <= 0.0) {
        throw std::runtime_error("MapsComparison: origin map has a non-positive resolution");
    }
    // Different-resolution comparison is an optional bonus we do not implement.
    if (std::abs(res - res_t) > 1e-9) {
        throw std::runtime_error("MapsComparison: maps have different resolutions");
    }

    // Overlap of both maps' boundaries (half-open ranges).
    const double x0 = std::max(oc.boundaries.min_x.force_numerical_value_in(cm),
                               tc.boundaries.min_x.force_numerical_value_in(cm));
    const double x1 = std::min(oc.boundaries.max_x.force_numerical_value_in(cm),
                               tc.boundaries.max_x.force_numerical_value_in(cm));
    const double y0 = std::max(oc.boundaries.min_y.force_numerical_value_in(cm),
                               tc.boundaries.min_y.force_numerical_value_in(cm));
    const double y1 = std::min(oc.boundaries.max_y.force_numerical_value_in(cm),
                               tc.boundaries.max_y.force_numerical_value_in(cm));
    const double z0 = std::max(oc.boundaries.min_height.force_numerical_value_in(cm),
                               tc.boundaries.min_height.force_numerical_value_in(cm));
    const double z1 = std::min(oc.boundaries.max_height.force_numerical_value_in(cm),
                               tc.boundaries.max_height.force_numerical_value_in(cm));
    if (x0 >= x1 || y0 >= y1 || z0 >= z1) {
        return 0.0; // no shared region to compare
    }

    // Sample voxel centres aligned to the origin's grid (offset-aware).
    const double ox = oc.offset.x.force_numerical_value_in(cm);
    const double oy = oc.offset.y.force_numerical_value_in(cm);
    const double oz = oc.offset.z.force_numerical_value_in(cm);

    const long ix_lo = static_cast<long>(std::floor((x0 - ox) / res));
    const long ix_hi = static_cast<long>(std::ceil((x1 - ox) / res));
    const long iy_lo = static_cast<long>(std::floor((y0 - oy) / res));
    const long iy_hi = static_cast<long>(std::ceil((y1 - oy) / res));
    const long iz_lo = static_cast<long>(std::floor((z0 - oz) / res));
    const long iz_hi = static_cast<long>(std::ceil((z1 - oz) / res));

    long total = 0;
    long correct = 0;
    for (long ix = ix_lo; ix < ix_hi; ++ix) {
        const double cx = ox + (static_cast<double>(ix) + 0.5) * res;
        if (cx < x0 || cx >= x1) {
            continue;
        }
        for (long iy = iy_lo; iy < iy_hi; ++iy) {
            const double cy = oy + (static_cast<double>(iy) + 0.5) * res;
            if (cy < y0 || cy >= y1) {
                continue;
            }
            for (long iz = iz_lo; iz < iz_hi; ++iz) {
                const double cz = oz + (static_cast<double>(iz) + 0.5) * res;
                if (cz < z0 || cz >= z1) {
                    continue;
                }
                const Position3D centre{cx * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]};
                ++total;
                if (origin.atVoxel(centre) == target.atVoxel(centre)) {
                    ++correct;
                }
            }
        }
    }

    return (total > 0) ? (100.0 * static_cast<double>(correct) / static_cast<double>(total)) : 0.0;
}

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    std::vector<double> scores;
    scores.reserve(targets.size());
    for (IMap3D* target : targets) {
        if (target == nullptr) {
            throw std::invalid_argument("MapsComparison: null target map");
        }
        scores.push_back(compareOne(origin, *target));
    }
    return scores;
}

} // namespace drone_mapper
