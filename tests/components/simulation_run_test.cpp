#include <gtest/gtest.h>

#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include "test_support.h"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace drone_mapper;
using namespace drone_mapper::test_support;

namespace {

double xcm(const Position3D& p) { return p.x.force_numerical_value_in(cm); }
double ycm(const Position3D& p) { return p.y.force_numerical_value_in(cm); }
double zcm(const Position3D& p) { return p.z.force_numerical_value_in(cm); }
double hdeg(const Orientation& o) { return o.horizontal.force_numerical_value_in(deg); }

types::SimulationConfigData makeSim(const std::filesystem::path& map, const Position3D& start,
                                    double res_cm = 10.0, double angle_deg = 0.0) {
    types::SimulationConfigData s{};
    s.map_filename = map;
    s.map_resolution = res_cm * cm;
    s.initial_drone_position = start;
    s.initial_angle = angle_deg * horizontal_angle[deg];
    return s;
}

std::filesystem::path uniqueRunDir() {
    static std::atomic<unsigned> n{0};
    const std::filesystem::path d =
        std::filesystem::temp_directory_path() / "drone_mapper_runs" / std::to_string(n.fetch_add(1));
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

// --- MockGPS / MockMovement (the PDF folds these into the SimulationRun suite) ---

TEST(SimulationRun, MockGps_ReturnsAndUpdatesPose) {
    MockGPS gps(posCm(10, 20, 30), orient(45), 10 * cm);
    EXPECT_DOUBLE_EQ(xcm(gps.position()), 10.0);
    EXPECT_DOUBLE_EQ(zcm(gps.position()), 30.0);
    EXPECT_DOUBLE_EQ(hdeg(gps.heading()), 45.0);

    gps.setPosition(posCm(1, 2, 3));
    gps.setHeading(orient(90));
    EXPECT_DOUBLE_EQ(ycm(gps.position()), 2.0);
    EXPECT_DOUBLE_EQ(hdeg(gps.heading()), 90.0);
}

TEST(SimulationRun, MockMovement_AdvancesAlongHeading) {
    MockGPS gps(posCm(0, 0, 0), orient(0), 10 * cm); // facing +x
    MockMovement mover(gps, makeDrone(5, 90, 100, 100));
    mover.advance(30 * cm);
    EXPECT_NEAR(xcm(gps.position()), 30.0, 1e-9);
    EXPECT_NEAR(ycm(gps.position()), 0.0, 1e-9);
    EXPECT_NEAR(zcm(gps.position()), 0.0, 1e-9);
}

TEST(SimulationRun, MockMovement_RotateUpdatesHeading) {
    MockGPS gps(posCm(0, 0, 0), orient(0), 10 * cm);
    MockMovement mover(gps, makeDrone(5, 90, 100, 100)); // max_rotate 90
    mover.rotate(types::RotationDirection::Left, 45.0 * horizontal_angle[deg]);
    EXPECT_DOUBLE_EQ(hdeg(gps.heading()), 45.0); // Left is positive
    mover.rotate(types::RotationDirection::Right, 90.0 * horizontal_angle[deg]);
    EXPECT_DOUBLE_EQ(hdeg(gps.heading()), 315.0); // 45 - 90 = -45 -> normalised to [0,360)
}

TEST(SimulationRun, MockMovement_ElevateSignedZ) {
    MockGPS gps(posCm(0, 0, 50), orient(0), 10 * cm);
    MockMovement mover(gps, makeDrone(5, 90, 100, 100));
    mover.elevate(20 * cm);
    EXPECT_NEAR(zcm(gps.position()), 70.0, 1e-9);
    mover.elevate(-30 * cm); // elevate can be negative
    EXPECT_NEAR(zcm(gps.position()), 40.0, 1e-9);
}

TEST(SimulationRun, MockMovement_ClampsToMax) {
    MockGPS gps(posCm(0, 0, 0), orient(0), 10 * cm);
    MockMovement mover(gps, makeDrone(5, /*max_rot*/30, /*max_adv*/25, /*max_ele*/20));
    mover.advance(1000 * cm);
    EXPECT_NEAR(xcm(gps.position()), 25.0, 1e-9); // clamped to max_advance
    mover.elevate(1000 * cm);
    EXPECT_NEAR(zcm(gps.position()), 20.0, 1e-9); // clamped to max_elevate
    mover.rotate(types::RotationDirection::Left, 1000.0 * horizontal_angle[deg]);
    EXPECT_DOUBLE_EQ(hdeg(gps.heading()), 30.0); // clamped to max_rotate
}

TEST(SimulationRun, MockMovement_AdvanceAfterRotate) {
    MockGPS gps(posCm(0, 0, 0), orient(0), 10 * cm);
    MockMovement mover(gps, makeDrone(5, 90, 100, 100));
    mover.rotate(types::RotationDirection::Left, 90.0 * horizontal_angle[deg]); // now facing +y
    mover.advance(40 * cm);
    EXPECT_NEAR(xcm(gps.position()), 0.0, 1e-9);
    EXPECT_NEAR(ycm(gps.position()), 40.0, 1e-9);
}

// --- SimulationRun proper, built through the real factory on a temp map ---

TEST(SimulationRun, RunReturnsResultWithConfigs) {
    const auto map = writeTempNpy(makeArray(5, 5, 5)); // all-empty hidden map
    const auto sim = makeSim(map, posCm(25, 25, 25));
    const auto mission = makeMission(makeBounds(0, 50, 0, 50, 0, 50), /*max_steps*/2000);
    const auto out = uniqueRunDir();

    SimulationRunFactoryImpl factory;
    const types::SimulationResult result =
        factory.create(sim, mission, makeDrone(5), makeLidar(20, 60, 2.5, 2), out)->run();

    EXPECT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.output_map_file, out / "output_map.npy");
    EXPECT_EQ(result.simulation_config.map_filename, map);
    EXPECT_EQ(result.mission_config.max_steps, 2000u);
    EXPECT_GT(result.output_map_config.resolution.force_numerical_value_in(cm), 0.0);
}

TEST(SimulationRun, RunScoresCompletedRunInRange) {
    const auto map = writeTempNpy(makeArray(5, 5, 5));
    const auto sim = makeSim(map, posCm(25, 25, 25));
    const auto mission = makeMission(makeBounds(0, 50, 0, 50, 0, 50), 4000);
    const auto out = uniqueRunDir();

    SimulationRunFactoryImpl factory;
    const types::SimulationResult result =
        factory.create(sim, mission, makeDrone(5), makeLidar(20, 60, 2.5, 2), out)->run();

    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_NE(result.mission_results.front().status, types::MissionRunStatus::Error);
    EXPECT_GE(result.mission_score, 0.0);
    EXPECT_LE(result.mission_score, 100.0);
}

TEST(SimulationRun, RunScoresErrorAsMinusOne) {
    const auto map = writeTempNpy(makeArray(5, 5, 5));
    const auto sim = makeSim(map, posCm(25, 25, 25));
    // Inverted mission bounds -> MISSION_BOUNDARY_INVALID -> mission Error.
    auto mission = makeMission(makeBounds(50, 0, 50, 0, 50, 0), 100);
    const auto out = uniqueRunDir();

    SimulationRunFactoryImpl factory;
    const types::SimulationResult result =
        factory.create(sim, mission, makeDrone(5), makeLidar(20, 60, 2.5, 2), out)->run();

    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::Error);
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
}

TEST(SimulationRun, ResolutionStatusFromFactor) {
    const auto map = writeTempNpy(makeArray(3, 3, 3));
    const auto sim = makeSim(map, posCm(15, 15, 15));
    SimulationRunFactoryImpl factory;
    const auto runWithFactor = [&](double factor) {
        auto mission = makeMission(makeBounds(0, 30, 0, 30, 0, 30), 100, 10.0, factor);
        return factory.create(sim, mission, makeDrone(5), makeLidar(20, 40, 2.5, 2), uniqueRunDir())
            ->run()
            .resolution_request_status;
    };
    EXPECT_EQ(runWithFactor(0.5), types::ResolutionRequestStatus::IgnoredTooSmall);
    EXPECT_EQ(runWithFactor(1.0), types::ResolutionRequestStatus::Accepted);
    EXPECT_EQ(runWithFactor(2.0), types::ResolutionRequestStatus::Ignored);
}

TEST(SimulationRun, ResolutionTooSmallLoggedImmediately) {
    const auto map = writeTempNpy(makeArray(3, 3, 3));
    const auto sim = makeSim(map, posCm(15, 15, 15));
    auto mission = makeMission(makeBounds(0, 30, 0, 30, 0, 30), 100, 10.0, /*factor*/0.5);
    const auto out = uniqueRunDir();

    SimulationRunFactoryImpl factory;
    (void)factory.create(sim, mission, makeDrone(5), makeLidar(20, 40, 2.5, 2), out)->run();

    const std::string log = readFile(out / "error_log.txt");
    EXPECT_NE(log.find("RESOLUTION_TOO_SMALL"), std::string::npos) << "log was: " << log;
}
