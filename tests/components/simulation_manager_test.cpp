#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/SimulationManager.h>

#include "test_mocks.h"
#include "test_support.h"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace drone_mapper;
using namespace drone_mapper::test_support;
using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Throw;

namespace {

types::SimulationConfigData simCfg(const std::string& map_name) {
    types::SimulationConfigData s{};
    s.map_filename = map_name;
    s.map_resolution = 10 * cm;
    return s;
}

// A factory action that echoes the create() configs into a fixed-score result, so
// tests can verify which (sim, mission) produced each run.
std::unique_ptr<ISimulationRun> echoStub(const types::SimulationConfigData& sim,
                                         const types::MissionConfigData& mission,
                                         const types::DroneConfigData&,
                                         const types::LidarConfigData&,
                                         const std::filesystem::path&) {
    types::SimulationResult res{};
    res.simulation_config = sim;
    res.mission_config = mission;
    res.mission_score = 42.0;
    res.mission_results = {types::MissionRunResult{types::MissionRunStatus::Completed, 1, {}}};
    return std::make_unique<StubSimulationRun>(res);
}

types::SimulationCompositionData
composition(std::vector<std::tuple<types::SimulationConfigData, std::vector<types::MissionConfigData>>> groups,
            std::vector<types::DroneConfigData> drones,
            std::vector<types::LidarConfigData> lidars) {
    types::SimulationCompositionData c{};
    c.composition_file = "composition.yml";
    c.simulation_mission_groups = std::move(groups);
    c.drones = std::move(drones);
    c.lidars = std::move(lidars);
    return c;
}

std::filesystem::path outDir() {
    static std::atomic<unsigned> n{0};
    const std::filesystem::path d =
        std::filesystem::temp_directory_path() / "drone_mapper_sm" / std::to_string(n.fetch_add(1));
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

types::MissionConfigData mission() { return makeMission(makeBounds(0, 50, 0, 50, 0, 50)); }

} // namespace

TEST(SimulationManager, RunsCartesianProduct) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke(echoStub));
    // 1 group with 2 missions, 3 drones, 2 lidars -> 2*3*2 = 12 runs.
    EXPECT_CALL(*factory, create(_, _, _, _, _)).Times(12);

    const auto comp = composition(
        {{simCfg("m.npy"), {mission(), mission()}}},
        {makeDrone(), makeDrone(), makeDrone()},
        {makeLidar(), makeLidar()});

    SimulationManager mgr(std::move(factory));
    const types::SimulationManagerReport report = mgr.run(comp, outDir());

    EXPECT_EQ(report.runs.size(), 12u);
}

TEST(SimulationManager, ReportMetadata) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke(echoStub));
    const auto comp = composition({{simCfg("m.npy"), {mission()}}}, {makeDrone()}, {makeLidar()});

    SimulationManager mgr(std::move(factory));
    const types::SimulationManagerReport report = mgr.run(comp, outDir());

    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);
    EXPECT_FALSE(report.generated_at_utc.empty());
    EXPECT_NE(report.generated_at_utc.find('T'), std::string::npos); // ISO-8601-ish
    EXPECT_NE(report.generated_at_utc.find('Z'), std::string::npos);
}

TEST(SimulationManager, FactoryThrowBecomesMinusOneResult) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    // First (lidar0) run fails to create; the second (lidar1) succeeds.
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce(Throw(std::runtime_error("map missing")))
        .WillRepeatedly(Invoke(echoStub));

    const auto comp = composition({{simCfg("m.npy"), {mission()}}}, {makeDrone()},
                                  {makeLidar(), makeLidar()});

    SimulationManager mgr(std::move(factory));
    const types::SimulationManagerReport report = mgr.run(comp, outDir());

    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results.front().status, types::MissionRunStatus::Error);
    EXPECT_EQ(report.runs[0].mission_results.front().errors.front().code, "RUN_FAILED");
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, 42.0); // other run unaffected
}

TEST(SimulationManager, NestedGroupsIterated) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Invoke(echoStub));
    const auto comp = composition(
        {{simCfg("alpha.npy"), {mission()}}, {simCfg("beta.npy"), {mission()}}},
        {makeDrone()}, {makeLidar()});

    SimulationManager mgr(std::move(factory));
    const types::SimulationManagerReport report = mgr.run(comp, outDir());

    ASSERT_EQ(report.runs.size(), 2u);
    std::set<std::string> maps{report.runs[0].simulation_config.map_filename.string(),
                               report.runs[1].simulation_config.map_filename.string()};
    EXPECT_EQ(maps, (std::set<std::string>{"alpha.npy", "beta.npy"}));
}

TEST(SimulationManager, PerRunOutputDirsUnique) {
    std::vector<std::filesystem::path> dirs;
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _))
        .WillByDefault(Invoke([&dirs](const types::SimulationConfigData& sim,
                                      const types::MissionConfigData& mis,
                                      const types::DroneConfigData& dr,
                                      const types::LidarConfigData& li,
                                      const std::filesystem::path& out) {
            dirs.push_back(out);
            return echoStub(sim, mis, dr, li, out);
        }));
    const auto comp = composition({{simCfg("m.npy"), {mission(), mission()}}},
                                  {makeDrone(), makeDrone()}, {makeLidar()});

    SimulationManager mgr(std::move(factory));
    (void)mgr.run(comp, outDir());

    ASSERT_EQ(dirs.size(), 4u); // 2 missions * 2 drones * 1 lidar
    const std::set<std::filesystem::path> unique(dirs.begin(), dirs.end());
    EXPECT_EQ(unique.size(), dirs.size()); // every run got its own output dir
}

TEST(SimulationManager, FactoryFailureLoggedToFile) {
    auto factory = std::make_unique<NiceMock<MockSimulationRunFactory>>();
    ON_CALL(*factory, create(_, _, _, _, _)).WillByDefault(Throw(std::runtime_error("map missing")));
    const auto comp = composition({{simCfg("mapA.npy"), {mission()}}}, {makeDrone()}, {makeLidar()});
    const auto out = outDir();

    SimulationManager mgr(std::move(factory));
    (void)mgr.run(comp, out);

    const std::filesystem::path log =
        out / "output_results" / "sim0_mapA" / "mission0" / "drone0__lidar0" / "error_log.txt";
    EXPECT_NE(readFile(log).find("RUN_FAILED"), std::string::npos) << "expected RUN_FAILED in " << log;
}
