#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>
#include <string>

namespace drone_mapper {

// Emits the hierarchical simulation_output.yaml (the PDF "score_report"): the
// flat SimulationManagerReport runs are regrouped by simulation -> mission ->
// (drone, lidar) run, and a summary block (totals + average/min/max score) is
// computed. The composition file path is passed in because the report struct no
// longer carries it.
class SimulationReportWriter {
public:
    static void write(const types::SimulationManagerReport& report,
                      const std::filesystem::path& composition_file,
                      const std::filesystem::path& output_file);

private:
    [[nodiscard]] static std::string statusString(types::MissionRunStatus status);
    [[nodiscard]] static std::string resolutionStatusString(types::ResolutionRequestStatus status);
    // A run counts as scored (vs errored) when its score is non-negative.
    [[nodiscard]] static bool isScored(const types::SimulationResult& result);
};

} // namespace drone_mapper
