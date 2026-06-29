#include <gtest/gtest.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>

#include "test_support.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <set>
#include <utility>
#include <vector>

using namespace drone_mapper;
using namespace drone_mapper::test_support;

namespace {

// An int8 output map (all Unmapped) the test stages with set(...) to act as the
// drone's knowledge. Bounds [0, 10*n)^3 at res 10 -> n^3 cells.
Map3DImpl makeOutput(double max_cm = 50.0, double res = 10.0) {
    return Map3DImpl(std::make_shared<NpyArray>(),
                     makeConfig(makeBounds(0, max_cm, 0, max_cm, 0, max_cm), res));
}

void applyMovement(MockMovement& mover, const types::MovementCommand& c) {
    switch (c.type) {
    case types::MovementCommandType::Rotate: mover.rotate(c.rotation, c.angle); break;
    case types::MovementCommandType::Advance: mover.advance(c.distance); break;
    case types::MovementCommandType::Elevate: mover.elevate(c.distance); break;
    case types::MovementCommandType::Hover: break;
    }
}

struct DriveResult {
    std::vector<types::MovementCommand> moves;
    types::DroneState final_state{};
    double max_x_cm = 0.0;
    bool finished = false;
};

// Runs the algorithm against a MockGPS/MockMovement: applies each emitted move to
// the actuator and feeds the updated state back. Scans are no-ops here (the
// output map is already staged), isolating the planner from the real lidar.
DriveResult drive(IMappingAlgorithm& algo, MockGPS& gps, MockMovement& mover,
                  std::size_t max_iters = 4000) {
    DriveResult r;
    types::DroneState st{gps.position(), gps.heading(), 0};
    r.max_x_cm = st.position.x.force_numerical_value_in(cm);
    for (std::size_t i = 0; i < max_iters; ++i) {
        const types::MappingStepCommand cmd = algo.nextStep(st, nullptr);
        if (cmd.status == types::AlgorithmStatus::Finished ||
            cmd.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
            r.finished = true;
            break;
        }
        if (cmd.movement.has_value()) {
            r.moves.push_back(*cmd.movement);
            applyMovement(mover, *cmd.movement);
        }
        st = types::DroneState{gps.position(), gps.heading(), st.step_index + 1};
        r.max_x_cm = std::max(r.max_x_cm, st.position.x.force_numerical_value_in(cm));
    }
    r.final_state = st;
    return r;
}

} // namespace

TEST(MappingAlgorithm, FirstStepRequestsScan) {
    Map3DImpl out = makeOutput();
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                              makeLidar(), makeDrone(1.0), out);

    const types::DroneState st{posCm(25, 25, 25), orient(0), 0};
    const types::MappingStepCommand cmd = algo.nextStep(st, nullptr);

    EXPECT_TRUE(cmd.scan_orientation.has_value());
    EXPECT_FALSE(cmd.movement.has_value());
    EXPECT_EQ(cmd.status, types::AlgorithmStatus::Working);
}

TEST(MappingAlgorithm, ScansSixDirectionsBeforeMoving) {
    Map3DImpl out = makeOutput();
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                              makeLidar(), makeDrone(1.0), out);

    const types::DroneState st{posCm(25, 25, 25), orient(0), 0}; // stationary
    std::set<std::pair<double, double>> dirs;
    for (int i = 0; i < 6; ++i) {
        const types::MappingStepCommand cmd = algo.nextStep(st, nullptr);
        ASSERT_TRUE(cmd.scan_orientation.has_value()) << "command " << i;
        EXPECT_FALSE(cmd.movement.has_value()) << "command " << i << " must not move before scanning";
        dirs.insert({cmd.scan_orientation->horizontal.force_numerical_value_in(deg),
                     cmd.scan_orientation->altitude.force_numerical_value_in(deg)});
    }
    const std::set<std::pair<double, double>> expected{
        {0, 0}, {90, 0}, {180, 0}, {270, 0}, {0, 60}, {0, -60}};
    EXPECT_EQ(dirs, expected);
}

TEST(MappingAlgorithm, FinishesWhenNothingNavigable) {
    Map3DImpl out = makeOutput(); // all Unmapped: nothing is Empty/passable
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                              makeLidar(), makeDrone(1.0), out);

    const types::DroneState st{posCm(25, 25, 25), orient(0), 0};
    bool finished = false;
    bool moved = false;
    for (int i = 0; i < 30 && !finished; ++i) {
        const types::MappingStepCommand cmd = algo.nextStep(st, nullptr);
        moved = moved || cmd.movement.has_value();
        finished = cmd.status == types::AlgorithmStatus::Finished;
    }
    EXPECT_TRUE(finished);
    EXPECT_FALSE(moved); // never tried to move where nothing is navigable
}

TEST(MappingAlgorithm, NavigatesIntoEmptyNeighbour) {
    Map3DImpl out = makeOutput();
    out.set(posCm(35, 25, 25), types::VoxelOccupancy::Empty); // +x neighbour, Empty + fits
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                              makeLidar(), makeDrone(1.0), out);

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm); // facing +x at the target
    MockMovement mover(gps, makeDrone(1.0));
    const DriveResult r = drive(algo, gps, mover);

    ASSERT_FALSE(r.moves.empty());
    // Facing +x straight at the +x neighbour, the first move is an Advance.
    EXPECT_EQ(r.moves.front().type, types::MovementCommandType::Advance);
}

TEST(MappingAlgorithm, OnlyEmptyIsPassable) {
    Map3DImpl out = makeOutput();
    out.set(posCm(35, 25, 25), types::VoxelOccupancy::Occupied); // +x neighbour blocked
    // every other neighbour is Unmapped (also not passable)
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                              makeLidar(), makeDrone(1.0), out);

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, makeDrone(1.0));
    const DriveResult r = drive(algo, gps, mover);

    EXPECT_TRUE(r.moves.empty()); // neither Occupied nor Unmapped is navigated into
    EXPECT_TRUE(r.finished);
}

TEST(MappingAlgorithm, DroneSizeGatesNarrowGaps) {
    // A single Empty cell whose neighbours are Unmapped. A tiny drone fits and
    // navigates; a large drone's body sphere overlaps the Unmapped neighbours, so
    // canFitAt fails and it never moves.
    const auto stage = [] (Map3DImpl& m) { m.set(posCm(35, 25, 25), types::VoxelOccupancy::Empty); };

    Map3DImpl small_out = makeOutput();
    stage(small_out);
    MappingAlgorithmImpl small_algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                                    makeLidar(), makeDrone(1.0), small_out);
    MockGPS small_gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement small_mover(small_gps, makeDrone(1.0));
    const DriveResult small_r = drive(small_algo, small_gps, small_mover);

    Map3DImpl large_out = makeOutput();
    stage(large_out);
    MappingAlgorithmImpl large_algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                                    makeLidar(), makeDrone(15.0), large_out);
    MockGPS large_gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement large_mover(large_gps, makeDrone(15.0));
    const DriveResult large_r = drive(large_algo, large_gps, large_mover);

    EXPECT_FALSE(small_r.moves.empty()) << "small drone should fit and navigate";
    EXPECT_TRUE(large_r.moves.empty()) << "large drone should not fit the 1-voxel gap";
}

TEST(MappingAlgorithm, MovementCommandsRespectLimits) {
    Map3DImpl out = makeOutput(100.0); // 10^3 cells
    for (int ix = 3; ix <= 7; ++ix) {
        out.set(posCm(ix * 10.0 + 5, 25, 25), types::VoxelOccupancy::Empty); // +x corridor
    }
    const types::DroneConfigData drone = makeDrone(1.0, /*max_rot*/30.0, /*max_adv*/25.0, /*max_ele*/20.0);
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                              makeLidar(), drone, out);
    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, drone);
    const DriveResult r = drive(algo, gps, mover);

    ASSERT_FALSE(r.moves.empty());
    for (const types::MovementCommand& m : r.moves) {
        switch (m.type) {
        case types::MovementCommandType::Rotate:
            EXPECT_LE(m.angle.force_numerical_value_in(deg), 30.0 + 1e-9);
            break;
        case types::MovementCommandType::Advance:
            EXPECT_LE(m.distance.force_numerical_value_in(cm), 25.0 + 1e-9);
            break;
        case types::MovementCommandType::Elevate:
            EXPECT_LE(std::abs(m.distance.force_numerical_value_in(cm)), 20.0 + 1e-9);
            break;
        case types::MovementCommandType::Hover:
            break;
        }
    }
}

TEST(MappingAlgorithm, ReachesFarCellViaBfs) {
    Map3DImpl out = makeOutput(100.0);
    for (int ix = 3; ix <= 9; ++ix) {
        out.set(posCm(ix * 10.0 + 5, 25, 25), types::VoxelOccupancy::Empty); // long +x corridor
    }
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                              makeLidar(), makeDrone(1.0), out);
    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, makeDrone(1.0));
    const DriveResult r = drive(algo, gps, mover);

    // Cell index 9 centre is x=95; the driven drone should reach the far end.
    EXPECT_GE(r.max_x_cm, 94.0);
}

TEST(MappingAlgorithm, RotatesBeforeAdvancingToSideNeighbour) {
    // Drone faces +x; the only navigable neighbour is +y. The algorithm must rotate
    // to face the target before advancing (it cannot strafe). So the first move is a
    // Rotate and an Advance only follows it.
    Map3DImpl out = makeOutput();
    out.set(posCm(25, 35, 25), types::VoxelOccupancy::Empty); // +y neighbour, Empty
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                              makeLidar(), makeDrone(1.0), out);
    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm); // facing +x
    MockMovement mover(gps, makeDrone(1.0));
    const DriveResult r = drive(algo, gps, mover);

    ASSERT_FALSE(r.moves.empty());
    EXPECT_EQ(r.moves.front().type, types::MovementCommandType::Rotate); // turn first
    const bool advanced = std::any_of(r.moves.begin(), r.moves.end(),
        [](const types::MovementCommand& m) { return m.type == types::MovementCommandType::Advance; });
    EXPECT_TRUE(advanced); // and advance only after having rotated
}

TEST(MappingAlgorithm, UnreachableEmptyCellFinishesWithoutLooping) {
    // A lone Empty cell with no Empty path to it (its neighbours are Unmapped) is
    // unreachable. The algorithm must recognise this and finish rather than spin
    // forever; drive() returns finished only if nextStep reported Finished within
    // the iteration cap.
    Map3DImpl out = makeOutput(80.0); // 8^3 cells
    out.set(posCm(55, 25, 25), types::VoxelOccupancy::Empty); // isolated, no Empty path from start
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 80, 0, 80, 0, 80)),
                              makeLidar(), makeDrone(1.0), out);
    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, makeDrone(1.0));
    const DriveResult r = drive(algo, gps, mover);

    EXPECT_TRUE(r.finished);       // terminated (no infinite loop)
    EXPECT_TRUE(r.moves.empty());  // never moved toward the unreachable cell
}

TEST(MappingAlgorithm, ElevateUsedForVerticalNeighbour) {
    Map3DImpl out = makeOutput();
    out.set(posCm(25, 25, 35), types::VoxelOccupancy::Empty); // +z neighbour, Empty
    MappingAlgorithmImpl algo(makeMission(makeBounds(0, 50, 0, 50, 0, 50)),
                              makeLidar(), makeDrone(1.0), out);
    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, makeDrone(1.0));
    const DriveResult r = drive(algo, gps, mover);

    const bool elevated = std::any_of(r.moves.begin(), r.moves.end(),
        [](const types::MovementCommand& m) { return m.type == types::MovementCommandType::Elevate; });
    EXPECT_TRUE(elevated);
}
