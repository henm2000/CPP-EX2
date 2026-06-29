#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>

#include "test_mocks.h"
#include "test_support.h"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

using namespace drone_mapper;
using namespace drone_mapper::test_support;
using testing::NiceMock;
using testing::Return;

namespace {

types::DroneStepResult cont() { return {types::DroneStepStatus::Continue, {}}; }
types::DroneStepResult done() { return {types::DroneStepStatus::Completed, "done"}; }
types::DroneStepResult fail(const std::string& m) { return {types::DroneStepStatus::Error, m}; }
types::DroneState stateAt(double x, double y, double z) { return {posCm(x, y, z), orient(0), 0}; }

Map3DImpl hiddenMap(const std::vector<std::array<std::size_t, 3>>& occupied = {}) {
    return Map3DImpl(makeArray(5, 5, 5, occupied), makeConfig(makeBounds(0, 50, 0, 50, 0, 50), 10));
}
Map3DImpl outputMap() {
    return Map3DImpl(std::make_shared<NpyArray>(), makeConfig(makeBounds(0, 50, 0, 50, 0, 50), 10));
}
std::filesystem::path runDir() {
    static std::atomic<unsigned> n{0};
    const std::filesystem::path d =
        std::filesystem::temp_directory_path() / "drone_mapper_mc" / std::to_string(n.fetch_add(1));
    std::error_code ec;
    std::filesystem::remove_all(d, ec);
    std::filesystem::create_directories(d, ec);
    return d;
}
std::string readFile(const std::filesystem::path& p) {
    std::ifstream in(p);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

TEST(MissionControl, CompletesWhenDroneFinishes) {
    Map3DImpl hidden = hiddenMap();
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    ON_CALL(dc, state()).WillByDefault(Return(stateAt(25, 25, 25)));
    EXPECT_CALL(dc, step()).WillOnce(Return(cont())).WillOnce(Return(done()));
    const auto dir = runDir();

    MissionControlImpl mc(makeMission(makeBounds(0, 50, 0, 50, 0, 50), 10), makeDrone(5),
                          hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(r.steps, 2u);
    EXPECT_TRUE(std::filesystem::exists(dir / "output_map.npy")); // output map saved
}

TEST(MissionControl, MaxStepsWhenNeverCompletes) {
    Map3DImpl hidden = hiddenMap();
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    ON_CALL(dc, state()).WillByDefault(Return(stateAt(25, 25, 25)));
    ON_CALL(dc, step()).WillByDefault(Return(cont())); // never completes
    const auto dir = runDir();

    MissionControlImpl mc(makeMission(makeBounds(0, 50, 0, 50, 0, 50), 5), makeDrone(5),
                          hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(r.steps, 5u);
}

TEST(MissionControl, DroneStepErrorPropagates) {
    Map3DImpl hidden = hiddenMap();
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    ON_CALL(dc, state()).WillByDefault(Return(stateAt(25, 25, 25)));
    ON_CALL(dc, step()).WillByDefault(Return(fail("actuator jammed")));
    const auto dir = runDir();

    MissionControlImpl mc(makeMission(makeBounds(0, 50, 0, 50, 0, 50), 10), makeDrone(5),
                          hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Error);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "DRONE_STEP_FAILED");
}

TEST(MissionControl, DetectsCollisionWhileMoving) {
    Map3DImpl hidden = hiddenMap({{4, 2, 2}}); // wall at world (45,25,25)
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    // Starts in free space, then a step moves the body into the wall cell.
    EXPECT_CALL(dc, state())
        .WillOnce(Return(stateAt(25, 25, 25)))
        .WillRepeatedly(Return(stateAt(45, 25, 25)));
    ON_CALL(dc, step()).WillByDefault(Return(cont()));
    const auto dir = runDir();

    // Radius 15 (benchmark size): the body sphere actually spans whole voxels, so
    // the swept collision check samples the wall cell.
    MissionControlImpl mc(makeMission(makeBounds(0, 50, 0, 50, 0, 50), 10), makeDrone(15),
                          hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Error);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "DRONE_HITS_OBSTACLE");
}

TEST(MissionControl, DetectsCollisionMidSegmentNotJustEndpoints) {
    // Wall cell (4,2,2) -> world (45,25,25). The drone moves x=25 -> x=65 in ONE
    // step; both endpoints (cells 2 and 6) are free, so only the swept sub-stepping
    // catches the wall sitting midway. An endpoint-only check would pass it.
    Map3DImpl hidden(makeArray(9, 5, 5, {{4, 2, 2}}),
                     makeConfig(makeBounds(0, 90, 0, 50, 0, 50), 10));
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    EXPECT_CALL(dc, state())
        .WillOnce(Return(stateAt(25, 25, 25)))   // start: free
        .WillRepeatedly(Return(stateAt(65, 25, 25))); // end: also free
    ON_CALL(dc, step()).WillByDefault(Return(cont()));
    const auto dir = runDir();

    MissionControlImpl mc(makeMission(makeBounds(0, 90, 0, 50, 0, 50), 10), makeDrone(8),
                          hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Error);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "DRONE_HITS_OBSTACLE");
}

TEST(MissionControl, SphericalBodyClearsBoxCornerObstacle) {
    // Drone (radius 10.5) centred at (25,25,25). An obstacle at the diagonal "box
    // corner" cell (3,3,3) is outside the spherical body, so the run proceeds; the
    // same-distance-class obstacle on a face axis (cell (3,2,2)) IS inside the
    // sphere and collides. This pins spherical collision, not a bounding box.
    NiceMock<MockDroneControl> dc;
    ON_CALL(dc, state()).WillByDefault(Return(stateAt(25, 25, 25)));

    // Corner obstacle only: the sphere never reaches it -> run completes.
    {
        Map3DImpl hidden(makeArray(6, 6, 6, {{3, 3, 3}}),
                         makeConfig(makeBounds(0, 60, 0, 60, 0, 60), 10));
        Map3DImpl output = outputMap();
        EXPECT_CALL(dc, step()).WillOnce(Return(done()));
        const auto dir = runDir();
        MissionControlImpl mc(makeMission(makeBounds(0, 60, 0, 60, 0, 60), 10), makeDrone(10.5),
                              hidden, output, dc, dir / "output_map.npy");
        const types::MissionRunResult r = mc.runMission();
        EXPECT_EQ(r.status, types::MissionRunStatus::Completed);
        EXPECT_TRUE(r.errors.empty());
    }

    // Face-axis obstacle within the sphere: collides immediately.
    {
        Map3DImpl hidden(makeArray(6, 6, 6, {{3, 2, 2}}),
                         makeConfig(makeBounds(0, 60, 0, 60, 0, 60), 10));
        Map3DImpl output = outputMap();
        const auto dir = runDir();
        MissionControlImpl mc(makeMission(makeBounds(0, 60, 0, 60, 0, 60), 10), makeDrone(10.5),
                              hidden, output, dc, dir / "output_map.npy");
        const types::MissionRunResult r = mc.runMission();
        EXPECT_EQ(r.status, types::MissionRunStatus::Error);
        ASSERT_FALSE(r.errors.empty());
        EXPECT_EQ(r.errors.front().code, "DRONE_HITS_OBSTACLE");
    }
}

TEST(MissionControl, DetectsInitialPositionInsideObstacle) {
    Map3DImpl hidden = hiddenMap({{2, 2, 2}}); // wall at the start cell (25,25,25)
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    ON_CALL(dc, state()).WillByDefault(Return(stateAt(25, 25, 25)));
    const auto dir = runDir();

    MissionControlImpl mc(makeMission(makeBounds(0, 50, 0, 50, 0, 50), 10), makeDrone(15),
                          hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Error);
    EXPECT_EQ(r.steps, 0u);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "DRONE_HITS_OBSTACLE");
}

TEST(MissionControl, ReportsInvalidMissionBounds) {
    Map3DImpl hidden = hiddenMap();
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    ON_CALL(dc, state()).WillByDefault(Return(stateAt(25, 25, 25)));
    EXPECT_CALL(dc, step()).Times(0); // never steps when bounds are invalid
    const auto dir = runDir();

    // Inverted bounds (max <= min).
    MissionControlImpl mc(makeMission(makeBounds(50, 0, 50, 0, 50, 0), 10), makeDrone(5),
                          hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Error);
    EXPECT_EQ(r.steps, 0u);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "MISSION_BOUNDARY_INVALID");
}

TEST(MissionControl, ErrorLoggedImmediately) {
    Map3DImpl hidden = hiddenMap();
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    ON_CALL(dc, state()).WillByDefault(Return(stateAt(25, 25, 25)));
    const auto dir = runDir();

    MissionControlImpl mc(makeMission(makeBounds(50, 0, 50, 0, 50, 0), 10), makeDrone(5),
                          hidden, output, dc, dir / "output_map.npy");
    (void)mc.runMission();

    const std::string log = readFile(dir / "error_log.txt");
    EXPECT_NE(log.find("MISSION_BOUNDARY_INVALID"), std::string::npos) << "log was: " << log;
}

TEST(MissionControl, NoCollisionInOpenMap) {
    Map3DImpl hidden = hiddenMap(); // fully empty
    Map3DImpl output = outputMap();
    NiceMock<MockDroneControl> dc;
    // Wander through several free cells, then finish.
    EXPECT_CALL(dc, state())
        .WillOnce(Return(stateAt(15, 15, 15)))
        .WillOnce(Return(stateAt(25, 15, 15)))
        .WillOnce(Return(stateAt(25, 25, 15)))
        .WillRepeatedly(Return(stateAt(35, 25, 15)));
    EXPECT_CALL(dc, step()).WillOnce(Return(cont())).WillOnce(Return(cont())).WillOnce(Return(done()));
    const auto dir = runDir();

    MissionControlImpl mc(makeMission(makeBounds(0, 50, 0, 50, 0, 50), 50), makeDrone(5),
                          hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Completed); // never DRONE_HITS_OBSTACLE
    EXPECT_TRUE(r.errors.empty());
}
