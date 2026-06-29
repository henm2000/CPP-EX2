#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>

#include "test_mocks.h"
#include "test_support.h"

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

using namespace drone_mapper;
using namespace drone_mapper::test_support;
using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

namespace {

types::MappingStepCommand scanCmd(const Orientation& o) {
    types::MappingStepCommand c;
    c.scan_orientation = o;
    c.status = types::AlgorithmStatus::Working;
    return c;
}
types::MappingStepCommand moveCmd(const types::MovementCommand& m) {
    types::MappingStepCommand c;
    c.movement = m;
    c.status = types::AlgorithmStatus::Working;
    return c;
}
types::MappingStepCommand noopCmd() {
    return types::MappingStepCommand{}; // no movement, no scan, status Working
}
types::MappingStepCommand finishedCmd() {
    types::MappingStepCommand c;
    c.status = types::AlgorithmStatus::Finished;
    return c;
}
types::MovementCommand advance(double cm_dist) {
    types::MovementCommand m;
    m.type = types::MovementCommandType::Advance;
    m.distance = cm_dist * cm;
    return m;
}

Map3DImpl outputMap(double max_cm = 100.0) {
    return Map3DImpl(std::make_shared<NpyArray>(), makeConfig(makeBounds(0, max_cm, 0, max_cm, 0, max_cm), 10));
}

// Counts output voxels the lidar could have written (Occupied/PotentiallyOccupied);
// markBodyEmpty never writes these, so a positive count isolates the scan.
std::size_t countObstacleCells(const Map3DImpl& map, int n = 10) {
    std::size_t count = 0;
    for (int x = 0; x < n; ++x) {
        for (int y = 0; y < n; ++y) {
            for (int z = 0; z < n; ++z) {
                const auto v = map.atVoxel(posCm(x * 10.0 + 5, y * 10.0 + 5, z * 10.0 + 5));
                if (v == types::VoxelOccupancy::Occupied || v == types::VoxelOccupancy::PotentiallyOccupied) {
                    ++count;
                }
            }
        }
    }
    return count;
}

} // namespace

TEST(DroneControl, ScanCommandAppliesToOutputMap) {
    // Hidden wall slab at x in [50,60); a +x scan from x=25 hits it and the
    // converter writes an Occupied cell into the output map.
    std::vector<std::array<std::size_t, 3>> wall;
    for (std::size_t y = 0; y < 10; ++y)
        for (std::size_t z = 0; z < 10; ++z) wall.push_back({5, y, z});
    Map3DImpl hidden(makeArray(10, 10, 10, wall), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output = outputMap();
    const auto drone = makeDrone(5);
    const auto lidarcfg = makeLidar(20, 120, 2.5, 1);

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm); // facing +x at the wall
    MockMovement mover(gps, drone);
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)), lidarcfg, drone, output);
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(scanCmd(orient(0))));

    DroneControlImpl dc(drone, makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                        lidar, gps, mover, output, algo);
    const types::DroneStepResult r = dc.step();

    EXPECT_EQ(r.status, types::DroneStepStatus::Continue);
    EXPECT_GT(countObstacleCells(output), 0u);
}

TEST(DroneControl, MoveCommandMovesDrone) {
    Map3DImpl hidden(makeArray(10, 10, 10), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output = outputMap();
    const auto drone = makeDrone(5, 90, 50, 50);
    const auto lidarcfg = makeLidar();

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm); // facing +x
    MockMovement mover(gps, drone);
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)), lidarcfg, drone, output);
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(moveCmd(advance(30))));

    DroneControlImpl dc(drone, makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                        lidar, gps, mover, output, algo);
    const types::DroneStepResult r = dc.step();

    EXPECT_EQ(r.status, types::DroneStepStatus::Continue);
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 55.0, 1e-9); // advanced +30 along +x
}

TEST(DroneControl, MovementFailureReturnsError) {
    Map3DImpl hidden(makeArray(10, 10, 10), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output = outputMap();
    const auto drone = makeDrone(5);
    const auto lidarcfg = makeLidar();

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    NiceMock<MockDroneMovement> mover; // forced failure
    ON_CALL(mover, advance(_)).WillByDefault(Return(types::MovementResult{false, "rotor stalled"}));
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)), lidarcfg, drone, output);
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(moveCmd(advance(30))));

    DroneControlImpl dc(drone, makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                        lidar, gps, mover, output, algo);
    const types::DroneStepResult r = dc.step();

    EXPECT_EQ(r.status, types::DroneStepStatus::Error);
    EXPECT_EQ(r.message, "rotor stalled");
}

TEST(DroneControl, FinishedCommandReturnsCompleted) {
    Map3DImpl hidden(makeArray(10, 10, 10), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output = outputMap();
    const auto drone = makeDrone(5);
    const auto lidarcfg = makeLidar();

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, drone);
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)), lidarcfg, drone, output);
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(finishedCmd()));

    DroneControlImpl dc(drone, makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                        lidar, gps, mover, output, algo);
    EXPECT_EQ(dc.step().status, types::DroneStepStatus::Completed);
}

TEST(DroneControl, MarksBodyFootprintEmpty) {
    Map3DImpl hidden(makeArray(10, 10, 10), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output = outputMap();
    const auto drone = makeDrone(5);
    const auto lidarcfg = makeLidar();

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, drone);
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)), lidarcfg, drone, output);
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(noopCmd())); // no scan, no move

    EXPECT_EQ(output.atVoxel(posCm(25, 25, 25)), types::VoxelOccupancy::Unmapped); // before
    DroneControlImpl dc(drone, makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                        lidar, gps, mover, output, algo);
    (void)dc.step();
    EXPECT_EQ(output.atVoxel(posCm(25, 25, 25)), types::VoxelOccupancy::Empty); // footprint marked
}

TEST(DroneControl, StateReflectsGpsAndStepIndex) {
    Map3DImpl hidden(makeArray(10, 10, 10), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output = outputMap();
    const auto drone = makeDrone(5);
    const auto lidarcfg = makeLidar();

    MockGPS gps(posCm(25, 35, 45), orient(90), 10 * cm);
    MockMovement mover(gps, drone);
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)), lidarcfg, drone, output);
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(noopCmd()));

    DroneControlImpl dc(drone, makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                        lidar, gps, mover, output, algo);

    const types::DroneState before = dc.state();
    EXPECT_EQ(before.step_index, 0u);
    EXPECT_DOUBLE_EQ(before.position.y.force_numerical_value_in(cm), 35.0);
    EXPECT_DOUBLE_EQ(before.heading.horizontal.force_numerical_value_in(deg), 90.0);

    (void)dc.step();
    EXPECT_EQ(dc.state().step_index, 1u); // increments per step
}

TEST(DroneControl, LatestScanFedBack) {
    Map3DImpl hidden(makeArray(10, 10, 10), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output = outputMap();
    const auto drone = makeDrone(5);
    const auto lidarcfg = makeLidar();

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, drone);
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(makeMission(makeBounds(0, 100, 0, 100, 0, 100)), lidarcfg, drone, output);

    const types::LidarScanResult* captured = nullptr;
    bool captured_set = false;
    EXPECT_CALL(algo, nextStep(_, _))
        .WillOnce(Return(scanCmd(orient(0))))            // step 1: ask for a scan (latest is null)
        .WillOnce(Invoke([&](const types::DroneState&, const types::LidarScanResult* latest) {
            captured = latest;                            // step 2: capture the fed-back scan
            captured_set = true;
            return noopCmd();
        }));

    DroneControlImpl dc(drone, makeMission(makeBounds(0, 100, 0, 100, 0, 100)),
                        lidar, gps, mover, output, algo);
    (void)dc.step();
    (void)dc.step();

    ASSERT_TRUE(captured_set);
    ASSERT_NE(captured, nullptr);          // the scan from step 1 was fed back
    EXPECT_FALSE(captured->empty());       // and it carries the beams that were fired
}
