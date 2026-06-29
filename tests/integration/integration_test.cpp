#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <TinyNPY.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationReportWriter.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include "test_mocks.h"
#include "test_support.h"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace drone_mapper;
using namespace drone_mapper::test_support;
using testing::_;
using testing::NiceMock;
using testing::Return;

namespace {

types::SimulationConfigData makeSim(const std::filesystem::path& map, const Position3D& start,
                                    double res_cm = 10.0) {
    types::SimulationConfigData s{};
    s.map_filename = map;
    s.map_resolution = res_cm * cm;
    s.initial_drone_position = start;
    s.initial_angle = 0.0 * horizontal_angle[deg];
    return s;
}

std::filesystem::path runDir() {
    static std::atomic<unsigned> n{0};
    const std::filesystem::path d =
        std::filesystem::temp_directory_path() / "drone_mapper_it" / std::to_string(n.fetch_add(1));
    std::error_code ec;
    std::filesystem::remove_all(d, ec);
    std::filesystem::create_directories(d, ec);
    return d;
}

std::shared_ptr<NpyArray> loadNpy(const std::filesystem::path& p) {
    auto a = std::make_shared<NpyArray>();
    const char* err = a->LoadNPY(p.string());
    EXPECT_EQ(err, nullptr) << "LoadNPY(" << p << "): " << (err ? err : "");
    return a;
}

std::size_t countMapped(const std::filesystem::path& p) {
    const auto a = loadNpy(p);
    const std::int8_t* d = a->Data<std::int8_t>();
    std::size_t c = 0;
    for (std::size_t i = 0; i < a->NumValue(); ++i) {
        if (d[i] != -1 && d[i] != -2) ++c; // not Unmapped, not OutOfBounds
    }
    return c;
}

bool sameArray(const std::filesystem::path& p1, const std::filesystem::path& p2) {
    const auto a = loadNpy(p1);
    const auto b = loadNpy(p2);
    if (a->NumValue() != b->NumValue()) return false;
    const std::int8_t* da = a->Data<std::int8_t>();
    const std::int8_t* db = b->Data<std::int8_t>();
    for (std::size_t i = 0; i < a->NumValue(); ++i) {
        if (da[i] != db[i]) return false;
    }
    return true;
}

std::string readFile(const std::filesystem::path& p) {
    std::ifstream in(p);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::size_t countObstacleCells(const Map3DImpl& map, int n) {
    std::size_t count = 0;
    for (int x = 0; x < n; ++x)
        for (int y = 0; y < n; ++y)
            for (int z = 0; z < n; ++z) {
                const auto v = map.atVoxel(posCm(x * 10.0 + 5, y * 10.0 + 5, z * 10.0 + 5));
                if (v == types::VoxelOccupancy::Occupied || v == types::VoxelOccupancy::PotentiallyOccupied)
                    ++count;
            }
    return count;
}

types::MappingStepCommand scanCmd(const Orientation& o) {
    types::MappingStepCommand c;
    c.scan_orientation = o;
    return c;
}
types::MappingStepCommand advanceCmd(double cm_dist) {
    types::MappingStepCommand c;
    c.movement = types::MovementCommand{types::MovementCommandType::Advance,
                                        types::RotationDirection::Left, {}, cm_dist * cm};
    return c;
}
types::MappingStepCommand finishedCmd() {
    types::MappingStepCommand c;
    c.status = types::AlgorithmStatus::Finished;
    return c;
}

} // namespace

TEST(Integration, RealAlgorithm_SingleVoxel_EndToEnd) {
    const auto comp_dir = runDir();
    types::SimulationCompositionData comp{};
    comp.composition_file = comp_dir / "composition.yml";
    comp.simulation_mission_groups = {
        {makeSim(dataMapPath("single_voxel_x4_y4_z4.npy"), posCm(25, 25, 25)),
         {makeMission(makeBounds(0, 50, 0, 50, 0, 50), 3000)}}};
    comp.drones = {makeDrone(5)};
    comp.lidars = {makeLidar(20, 60, 2.5, 3)};

    SimulationManager mgr(std::make_unique<SimulationRunFactoryImpl>());
    const types::SimulationManagerReport report = mgr.run(comp, comp_dir);
    const auto yaml = comp_dir / "simulation_output.yaml";
    SimulationReportWriter::write(report, comp.composition_file, yaml);

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_GE(report.runs[0].mission_score, 0.0); // a real run, not a -1 error
    // output_map.npy written and numpy-readable as int8
    EXPECT_TRUE(std::filesystem::exists(report.runs[0].output_map_file));
    EXPECT_EQ(loadNpy(report.runs[0].output_map_file)->Type(), 'i');
    // simulation_output.yaml well-formed enough to carry the score report
    const std::string y = readFile(yaml);
    EXPECT_NE(y.find("score_report"), std::string::npos);
    EXPECT_NE(y.find("status:"), std::string::npos);
}

TEST(Integration, RealAlgorithm_Benchmark_MapsAndScores) {
    SimulationRunFactoryImpl factory;
    const auto result =
        factory.create(makeSim(dataMapPath("benchmark_map.npy"), posCm(55, 55, 175)),
                       makeMission(makeBounds(0, 290, 0, 300, 0, 310), 8000),
                       makeDrone(10), makeLidar(20, 120, 2.5, 4), runDir())
            ->run();

    ASSERT_FALSE(result.mission_results.empty());
    EXPECT_NE(result.mission_results.front().status, types::MissionRunStatus::Error); // no collision
    EXPECT_GT(countMapped(result.output_map_file), 2000u);
    EXPECT_GT(result.mission_score, 0.0);
}

TEST(Integration, DroneSizeChangesCoverage_Benchmark) {
    // Three drone sizes matching the benchmark map's room-entrance widths: at 10cm
    // voxels, diameter 10/20/30cm (radius 5/10/15) fit through 1x1 / 2x2 / 3x3
    // voxel openings respectively. Each runs collision-free on the big map, maps a
    // non-trivial amount, and the three output maps are pairwise distinct -> drone
    // size measurably changes coverage. (Run to completion, smaller drones map
    // strictly more -- ~15.6k vs 15.5k vs 14.0k cells; this test caps max_steps for
    // speed, so it asserts the robust distinctness rather than that ordering, which
    // only stabilises once every size finishes exploring.)
    SimulationRunFactoryImpl factory;
    const auto mission = makeMission(makeBounds(0, 290, 0, 300, 0, 310), 8000);
    const auto lidar = makeLidar(20, 120, 2.5, 4);
    const Position3D start = posCm(55, 55, 175);

    const auto fits1x1 = factory.create(makeSim(dataMapPath("benchmark_map.npy"), start),
                                        mission, makeDrone(5), lidar, runDir())->run();
    const auto fits2x2 = factory.create(makeSim(dataMapPath("benchmark_map.npy"), start),
                                        mission, makeDrone(10), lidar, runDir())->run();
    const auto fits3x3 = factory.create(makeSim(dataMapPath("benchmark_map.npy"), start),
                                        mission, makeDrone(15), lidar, runDir())->run();

    for (const types::SimulationResult* r : {&fits1x1, &fits2x2, &fits3x3}) {
        EXPECT_NE(r->mission_results.front().status, types::MissionRunStatus::Error); // collision-free
        EXPECT_GT(countMapped(r->output_map_file), 2000u);
    }
    EXPECT_FALSE(sameArray(fits1x1.output_map_file, fits2x2.output_map_file));
    EXPECT_FALSE(sameArray(fits1x1.output_map_file, fits3x3.output_map_file));
    EXPECT_FALSE(sameArray(fits2x2.output_map_file, fits3x3.output_map_file));
}

TEST(Integration, MockAlgorithm_EndToEnd) {
    // Real DroneControl + MissionControl + MockGPS/Movement/Lidar, driven by a
    // scripted algorithm: scan the wall, advance, then finish.
    std::vector<std::array<std::size_t, 3>> wall;
    for (std::size_t y = 0; y < 10; ++y)
        for (std::size_t z = 0; z < 10; ++z) wall.push_back({7, y, z});
    Map3DImpl hidden(makeArray(10, 10, 10, wall), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output(std::make_shared<NpyArray>(), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    const auto drone = makeDrone(5);
    const auto lidarcfg = makeLidar(20, 120, 2.5, 1);
    const auto mission = makeMission(makeBounds(0, 100, 0, 100, 0, 100), 50);

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, drone);
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(mission, lidarcfg, drone, output);
    EXPECT_CALL(algo, nextStep(_, _))
        .WillOnce(Return(scanCmd(orient(0))))   // see the wall ahead
        .WillOnce(Return(advanceCmd(30)))       // drive +30 along +x
        .WillRepeatedly(Return(finishedCmd()));

    const auto dir = runDir();
    DroneControlImpl dc(drone, mission, lidar, gps, mover, output, algo);
    MissionControlImpl mc(mission, drone, hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Completed);
    EXPECT_GT(countObstacleCells(output, 10), 0u);                          // scripted scan applied
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 55.0, 1e-9); // GPS ended where driven
}

TEST(Integration, MockAlgorithm_CollisionReported) {
    std::vector<std::array<std::size_t, 3>> wall;
    for (std::size_t y = 0; y < 10; ++y)
        for (std::size_t z = 0; z < 10; ++z) wall.push_back({7, y, z}); // wall at x in [70,80)
    Map3DImpl hidden(makeArray(10, 10, 10, wall), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    Map3DImpl output(std::make_shared<NpyArray>(), makeConfig(makeBounds(0, 100, 0, 100, 0, 100), 10));
    const auto drone = makeDrone(15); // body actually spans voxels -> collides with the wall
    const auto lidarcfg = makeLidar(20, 120, 2.5, 1);
    const auto mission = makeMission(makeBounds(0, 100, 0, 100, 0, 100), 50);

    MockGPS gps(posCm(25, 25, 25), orient(0), 10 * cm);
    MockMovement mover(gps, drone);
    MockLidar lidar(lidarcfg, hidden, gps);
    NiceMock<MockMappingAlgorithm> algo(mission, lidarcfg, drone, output);
    ON_CALL(algo, nextStep(_, _)).WillByDefault(Return(advanceCmd(50))); // drive straight into the wall

    const auto dir = runDir();
    DroneControlImpl dc(drone, mission, lidar, gps, mover, output, algo);
    MissionControlImpl mc(mission, drone, hidden, output, dc, dir / "output_map.npy");
    const types::MissionRunResult r = mc.runMission();

    EXPECT_EQ(r.status, types::MissionRunStatus::Error);
    ASSERT_FALSE(r.errors.empty());
    EXPECT_EQ(r.errors.front().code, "DRONE_HITS_OBSTACLE");
    EXPECT_NE(readFile(dir / "error_log.txt").find("DRONE_HITS_OBSTACLE"), std::string::npos);
}

TEST(Integration, WholeGroupMapFailure_FilledMinusOne) {
    const auto dir = runDir();
    types::SimulationCompositionData comp{};
    comp.composition_file = dir / "composition.yml";
    comp.simulation_mission_groups = {
        {makeSim("/no/such/map_file.npy", posCm(25, 25, 25)),               // bad map -> RUN_FAILED
         {makeMission(makeBounds(0, 50, 0, 50, 0, 50), 100)}},
        {makeSim(dataMapPath("single_voxel_x4_y4_z4.npy"), posCm(25, 25, 25)), // good map
         {makeMission(makeBounds(0, 50, 0, 50, 0, 50), 3000)}}};
    comp.drones = {makeDrone(5)};
    comp.lidars = {makeLidar(20, 60, 2.5, 3)};

    SimulationManager mgr(std::make_unique<SimulationRunFactoryImpl>());
    const types::SimulationManagerReport report = mgr.run(comp, dir);

    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0); // bad-map group filled with -1
    EXPECT_GE(report.runs[1].mission_score, 0.0);          // other group still succeeds
}

TEST(Integration, OutputMapMatchesReportedScore) {
    SimulationRunFactoryImpl factory;
    const auto result =
        factory.create(makeSim(dataMapPath("single_voxel_x4_y4_z4.npy"), posCm(25, 25, 25)),
                       makeMission(makeBounds(0, 50, 0, 50, 0, 50), 3000),
                       makeDrone(5), makeLidar(20, 60, 2.5, 3), runDir())
            ->run();

    // Reload the produced output map + hidden map and recompute the score: it must
    // match what the run reported (and what the report writes to YAML).
    Map3DImpl produced(loadNpy(result.output_map_file), result.output_map_config);
    Map3DImpl hidden(loadNpy(result.simulation_config.map_filename),
                     makeConfig(makeBounds(0, 50, 0, 50, 0, 50), 10));
    const std::vector<double> scores = MapsComparison::compare(hidden, {&produced});

    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores.front(), result.mission_score);
}
