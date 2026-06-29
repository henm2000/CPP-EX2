#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/types/MapTypes.h>

#include "test_support.h"

#include <cstddef>
#include <memory>
#include <vector>

using namespace drone_mapper;
using namespace drone_mapper::test_support;

TEST(MapsComparison, IdenticalMapsReturn100) {
    auto array = makeArray(4, 4, 4, {{1, 1, 1}, {2, 2, 2}});
    const auto config = makeConfig(makeBounds(0, 40, 0, 40, 0, 40), 10);
    Map3DImpl origin(array, config);
    Map3DImpl target(array, config);

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

TEST(MapsComparison, OppositeMapsReturnZero) {
    std::vector<std::array<std::size_t, 3>> all;
    for (std::size_t x = 0; x < 3; ++x) {
        for (std::size_t y = 0; y < 3; ++y) {
            for (std::size_t z = 0; z < 3; ++z) {
                all.push_back({x, y, z});
            }
        }
    }
    auto empty = makeArray(3, 3, 3);
    auto full = makeArray(3, 3, 3, all);
    const auto config = makeConfig(makeBounds(0, 30, 0, 30, 0, 30), 10);
    Map3DImpl origin(empty, config);
    Map3DImpl target(full, config);

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    EXPECT_DOUBLE_EQ(scores.front(), 0.0);
}

TEST(MapsComparison, SimilarMapsAreHighButNotPerfect) {
    auto a = makeArray(4, 4, 4, {{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
    auto b = makeArray(4, 4, 4, {{0, 0, 0}, {1, 1, 1}, {2, 2, 2}}); // one voxel differs
    const auto config = makeConfig(makeBounds(0, 40, 0, 40, 0, 40), 10);
    Map3DImpl origin(a, config);
    Map3DImpl target(b, config);

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    EXPECT_GT(scores.front(), 90.0);
    EXPECT_LT(scores.front(), 100.0);
}

TEST(MapsComparison, DifferentResolutionsThrow) {
    auto array = makeArray(4, 4, 4);
    Map3DImpl origin(array, makeConfig(makeBounds(0, 40, 0, 40, 0, 40), 10));
    Map3DImpl target(array, makeConfig(makeBounds(0, 40, 0, 40, 0, 40), 20));
    EXPECT_THROW((void)MapsComparison::compare(origin, {&target}), std::exception);
}

TEST(MapsComparison, MultipleTargetsProduceMultipleScores) {
    auto array = makeArray(4, 4, 4, {{1, 1, 1}});
    const auto config = makeConfig(makeBounds(0, 40, 0, 40, 0, 40), 10);
    Map3DImpl origin(array, config);
    Map3DImpl t1(array, config);
    Map3DImpl t2(array, config);

    const std::vector<double> scores = MapsComparison::compare(origin, {&t1, &t2});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[1], 100.0);
}

TEST(MapsComparison, PartialOverlapBoundsScoredOnIntersectionOnly) {
    // Origin covers [0,40)^3 with an Occupied voxel at index (0,0,0) (centre
    // 5,5,5). Target covers [20,60)^3 (offset +20) and is all-empty. The bounds
    // overlap is [20,40)^3, which EXCLUDES the origin's Occupied voxel, so every
    // shared voxel is Empty in both maps -> 100. (A comparator that scored over
    // the origin's full bounds would sample the unmatched (0,0,0) voxel and fall
    // below 100, so this pins down "intersection only".)
    auto origin_arr = makeArray(4, 4, 4, {{0, 0, 0}});
    Map3DImpl origin(origin_arr, makeConfig(makeBounds(0, 40, 0, 40, 0, 40), 10));

    auto target_arr = makeArray(4, 4, 4);
    Map3DImpl target(target_arr,
                     makeConfig(makeBounds(20, 60, 20, 60, 20, 60), 10, posCm(20, 20, 20)));

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    EXPECT_DOUBLE_EQ(scores.front(), 100.0);
}

TEST(MapsComparison, HalfCorrectScoresFifty) {
    // 2x2x2 = 8 shared voxels. Hidden is all Empty; the output agrees (Empty) on
    // exactly 4 of them and is left Unmapped on the other 4 -> precisely 50.0.
    const auto config = makeConfig(makeBounds(0, 20, 0, 20, 0, 20), 10);
    auto hidden_arr = makeArray(2, 2, 2); // all Empty
    Map3DImpl hidden(hidden_arr, config);

    Map3DImpl output(std::make_shared<NpyArray>(), config); // int8, all Unmapped
    int set_empty = 0;
    for (std::size_t x = 0; x < 2; ++x) {
        for (std::size_t y = 0; y < 2; ++y) {
            for (std::size_t z = 0; z < 2; ++z) {
                if (set_empty++ < 4) {
                    output.set(posCm(10.0 * x + 5, 10.0 * y + 5, 10.0 * z + 5),
                               types::VoxelOccupancy::Empty);
                }
            }
        }
    }

    const std::vector<double> scores = MapsComparison::compare(hidden, {&output});
    EXPECT_DOUBLE_EQ(scores.front(), 50.0);
}

TEST(MapsComparison, UnmappedCountsAsWrong) {
    // Hidden map: all Empty. Output map (int8): all Empty except one cell left
    // Unmapped (-1). The single Unmapped cell must score as a mismatch against
    // Empty -> strictly below 100 (a comparator that collapsed Unmapped to Empty
    // would wrongly return 100).
    const auto config = makeConfig(makeBounds(0, 20, 0, 20, 0, 20), 10);
    auto hidden_arr = makeArray(2, 2, 2);
    Map3DImpl hidden(hidden_arr, config);

    Map3DImpl output(std::make_shared<NpyArray>(), config); // int8, starts all Unmapped
    for (std::size_t x = 0; x < 2; ++x) {
        for (std::size_t y = 0; y < 2; ++y) {
            for (std::size_t z = 0; z < 2; ++z) {
                output.set(posCm(10.0 * x + 5, 10.0 * y + 5, 10.0 * z + 5),
                           types::VoxelOccupancy::Empty);
            }
        }
    }
    output.set(posCm(5, 5, 5), types::VoxelOccupancy::Unmapped); // one cell stays Unmapped

    const std::vector<double> scores = MapsComparison::compare(hidden, {&output});
    EXPECT_LT(scores.front(), 100.0);
    EXPECT_GT(scores.front(), 0.0);
}

TEST(MapsComparison, PotentiallyOccupiedCountsAsWrong) {
    // -3 (PotentiallyOccupied, written by the course ScanResultToVoxels for a hit
    // closer than z_min) is a distinct sentinel matching neither Empty nor
    // Occupied. Hidden has one Occupied cell; the output marks that same cell
    // PotentiallyOccupied and agrees (Empty) everywhere else -> exactly one
    // mismatch, score below 100. (Collapsing -3 to Occupied would return 100.)
    const auto config = makeConfig(makeBounds(0, 20, 0, 20, 0, 20), 10);
    auto hidden_arr = makeArray(2, 2, 2, {{0, 0, 0}}); // one Occupied cell at centre 5,5,5
    Map3DImpl hidden(hidden_arr, config);

    Map3DImpl output(std::make_shared<NpyArray>(), config);
    for (std::size_t x = 0; x < 2; ++x) {
        for (std::size_t y = 0; y < 2; ++y) {
            for (std::size_t z = 0; z < 2; ++z) {
                output.set(posCm(10.0 * x + 5, 10.0 * y + 5, 10.0 * z + 5),
                           types::VoxelOccupancy::Empty);
            }
        }
    }
    output.set(posCm(5, 5, 5), types::VoxelOccupancy::PotentiallyOccupied);

    const std::vector<double> scores = MapsComparison::compare(hidden, {&output});
    EXPECT_LT(scores.front(), 100.0);
    EXPECT_GT(scores.front(), 0.0);
}
