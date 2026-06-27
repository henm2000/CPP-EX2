#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include "test_support.h"

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
