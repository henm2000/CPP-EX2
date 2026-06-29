#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>

#include "test_support.h"

#include <array>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

using namespace drone_mapper;
using namespace drone_mapper::test_support;

namespace {

// A hidden uint8 map (res 10cm, offset 0) with an optional solid Occupied wall
// slab spanning every y,z at x index `wall_ix`. A wall_ix past the grid leaves
// the map fully open. The wall is a full slab (not a single voxel) so a beam
// cannot slip past it through floating-point sampling.
Map3DImpl makeWallMap(std::size_t nx = 20, std::size_t ny = 20, std::size_t nz = 5,
                      std::size_t wall_ix = 1000) {
    std::vector<std::array<std::size_t, 3>> occupied;
    if (wall_ix < nx) {
        for (std::size_t y = 0; y < ny; ++y) {
            for (std::size_t z = 0; z < nz; ++z) {
                occupied.push_back({wall_ix, y, z});
            }
        }
    }
    auto array = makeArray(nx, ny, nz, occupied);
    return Map3DImpl(array, makeConfig(makeBounds(0, 10.0 * static_cast<double>(nx),
                                                  0, 10.0 * static_cast<double>(ny),
                                                  0, 10.0 * static_cast<double>(nz)),
                                       10));
}

constexpr double kMiss = std::numeric_limits<double>::max();

} // namespace

TEST(MockLidar, ReportsObstacleAheadWithinRange) {
    // Drone at x=55 facing +x; a wall fills x in [150,160). The centre beam should
    // report ~95cm, comfortably inside (z_min=20, z_max=200).
    Map3DImpl map = makeWallMap(20, 20, 5, /*wall_ix=*/15);
    MockGPS gps(posCm(55, 55, 25), orient(0), 10 * cm);
    MockLidar lidar(makeLidar(20, 200), map, gps);

    const auto scan = lidar.scan(orient(0));
    ASSERT_FALSE(scan.empty());
    const double d = scan.front().distance.force_numerical_value_in(cm);
    EXPECT_GT(d, 20.0);
    EXPECT_LT(d, 200.0);
}

TEST(MockLidar, MissReturnsMaxDistance) {
    // No obstacle along the beam. HW2's LidarHit has no optional: a miss is the
    // sentinel max-double centimetres, not an empty result.
    Map3DImpl map = makeWallMap(20, 20, 5, /*no wall=*/1000);
    MockGPS gps(posCm(55, 55, 25), orient(0), 10 * cm);
    MockLidar lidar(makeLidar(20, 200), map, gps);

    const auto scan = lidar.scan(orient(0));
    ASSERT_FALSE(scan.empty());
    EXPECT_DOUBLE_EQ(scan.front().distance.force_numerical_value_in(cm), kMiss);
}

TEST(MockLidar, TooCloseHitReturnsZero) {
    // Wall at x in [10,20); drone at x=5 is only ~5cm away, closer than z_min=20.
    // The exact cell is unknown at distance 0, so the beam reports 0cm.
    Map3DImpl map = makeWallMap(20, 20, 5, /*wall_ix=*/1);
    MockGPS gps(posCm(5, 55, 25), orient(0), 10 * cm);
    MockLidar lidar(makeLidar(20, 200), map, gps);

    const auto scan = lidar.scan(orient(0));
    ASSERT_FALSE(scan.empty());
    EXPECT_DOUBLE_EQ(scan.front().distance.force_numerical_value_in(cm), 0.0);
}

TEST(MockLidar, ZeroOrOneFovCircleFiresOnlyCenterBeam) {
    // Per ex1, circle 0 is always fired. fov_circles==0 and ==1 both yield exactly
    // ONE beam, carrying the relative scan orientation it was aimed with.
    Map3DImpl map = makeWallMap(); // open
    MockGPS gps(posCm(55, 55, 25), orient(0), 10 * cm);

    for (std::size_t fov : {std::size_t{0}, std::size_t{1}}) {
        MockLidar lidar(makeLidar(20, 200, 2.5, fov), map, gps);
        const auto scan = lidar.scan(orient(30, 10));
        ASSERT_EQ(scan.size(), 1u) << "fov_circles=" << fov;
        EXPECT_DOUBLE_EQ(scan.front().angle.horizontal.force_numerical_value_in(deg), 30.0);
        EXPECT_DOUBLE_EQ(scan.front().angle.altitude.force_numerical_value_in(deg), 10.0);
    }
}

TEST(MockLidar, BeamCountMatchesFovCircles) {
    // fov=N fires sum_{k=0}^{N-1} 4^k beams: 1, 5, 21, 85, ...
    Map3DImpl map = makeWallMap(); // open
    MockGPS gps(posCm(55, 55, 25), orient(0), 10 * cm);

    const std::array<std::pair<std::size_t, std::size_t>, 4> expected{{
        {1, 1}, {2, 5}, {3, 21}, {4, 85},
    }};
    for (const auto& [fov, count] : expected) {
        MockLidar lidar(makeLidar(20, 200, 2.5, fov), map, gps);
        EXPECT_EQ(lidar.scan(orient(0)).size(), count) << "fov_circles=" << fov;
    }
}

TEST(MockLidar, ScanIsRelativeToHeading) {
    // Solid wall to the east (+x). The same relative scan hits or misses depending
    // on heading, and re-aiming by -heading restores the hit: heading is added
    // exactly once (relative angle + heading).
    Map3DImpl map = makeWallMap(20, 20, 5, /*wall_ix=*/15);
    MockGPS gps(posCm(55, 55, 25), orient(0), 10 * cm);
    MockLidar lidar(makeLidar(20, 200), map, gps);

    // Facing east, relative scan 0 points straight at the wall -> finite hit.
    gps.setHeading(orient(0));
    EXPECT_LT(lidar.scan(orient(0)).front().distance.force_numerical_value_in(cm), kMiss);

    // Facing north, that same relative scan now points +y (no wall) -> miss.
    gps.setHeading(orient(90));
    EXPECT_DOUBLE_EQ(lidar.scan(orient(0)).front().distance.force_numerical_value_in(cm), kMiss);

    // Still facing north, relative scan -90 re-aims east at the wall -> finite hit.
    EXPECT_LT(lidar.scan(orient(-90)).front().distance.force_numerical_value_in(cm), kMiss);
}

TEST(MockLidar, ConfigGetterReturnsInjectedConfig) {
    Map3DImpl map = makeWallMap();
    MockGPS gps(posCm(55, 55, 25), orient(0), 10 * cm);
    MockLidar lidar(makeLidar(15, 250, 3.0, 3), map, gps);

    const auto got = lidar.config();
    EXPECT_DOUBLE_EQ(got.z_min.force_numerical_value_in(cm), 15.0);
    EXPECT_DOUBLE_EQ(got.z_max.force_numerical_value_in(cm), 250.0);
    EXPECT_DOUBLE_EQ(got.d.force_numerical_value_in(cm), 3.0);
    EXPECT_EQ(got.fov_circles, 3u);
}
