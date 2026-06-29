#pragma once

// Shared GMock test doubles for the drone_mapper component + integration tests.
// Component suites inject these (NiceMock + ON_CALL) so a seeded bug in another
// component cannot make the suite fail.

#include <gmock/gmock.h>

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>

#include <filesystem>
#include <memory>
#include <utility>

namespace drone_mapper::test_support {

// The ctor takes (mission, lidar, drone, const IMap3D&); inherit it so tests can
// construct the mock with a real staged output map.
class MockMappingAlgorithm : public IMappingAlgorithm {
public:
    using IMappingAlgorithm::IMappingAlgorithm;
    MOCK_METHOD(types::MappingStepCommand, nextStep,
                (const types::DroneState& state, const types::LidarScanResult* latest_scan),
                (override));
};

class MockDroneControl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

class MockDroneMovement : public IDroneMovement {
public:
    MOCK_METHOD(types::MovementResult, rotate,
                (types::RotationDirection direction, HorizontalAngle angle), (override));
    MOCK_METHOD(types::MovementResult, advance, (PhysicalLength distance), (override));
    MOCK_METHOD(types::MovementResult, elevate, (PhysicalLength distance), (override));
};

// ILidar double, used to assert DroneControl's movement-before-scan ordering
// without depending on the real ray-marching MockLidar.
class MockLidarSensor : public ILidar {
public:
    MOCK_METHOD(types::LidarScanResult, scan, (Orientation scan_orientation), (const, override));
    MOCK_METHOD(types::LidarConfigData, config, (), (const, override));
};

class MockMissionControl : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

class MockSimulationRunFactory : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>, create,
                (const types::SimulationConfigData& simulation,
                 const types::MissionConfigData& mission,
                 const types::DroneConfigData& drone,
                 const types::LidarConfigData& lidar,
                 const std::filesystem::path& output_path),
                (override));
};

// A trivial ISimulationRun that yields a fixed, pre-canned SimulationResult.
class StubSimulationRun final : public ISimulationRun {
public:
    explicit StubSimulationRun(types::SimulationResult result) : result_(std::move(result)) {}
    [[nodiscard]] types::SimulationResult run() override { return result_; }

private:
    types::SimulationResult result_;
};

} // namespace drone_mapper::test_support
