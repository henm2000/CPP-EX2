#include <drone_mapper/SimulationReportWriter.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace drone_mapper {

std::string SimulationReportWriter::statusString(types::MissionRunStatus status) {
    switch (status) {
    case types::MissionRunStatus::Completed:
        return "completed";
    case types::MissionRunStatus::MaxSteps:
        return "max_steps";
    case types::MissionRunStatus::Error:
        return "error";
    }
    return "unknown";
}

std::string SimulationReportWriter::resolutionStatusString(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted:
        return "ACCEPTED";
    case types::ResolutionRequestStatus::Ignored:
        return "IGNORED";
    case types::ResolutionRequestStatus::IgnoredTooSmall:
        return "IGNORED TOO SMALL";
    }
    return "IGNORED";
}

bool SimulationReportWriter::isScored(const types::SimulationResult& result) {
    return result.mission_score >= 0.0;
}

void SimulationReportWriter::write(const types::SimulationManagerReport& report,
                                   const std::filesystem::path& composition_file,
                                   const std::filesystem::path& output_file) {
    // Regroup the flat run list into simulation -> mission -> runs, preserving
    // first-seen order. Identity keys are derived from the carried configs.
    struct MissionGroup {
        const types::SimulationResult* sample = nullptr;
        std::string key;
        std::vector<const types::SimulationResult*> runs;
    };
    struct SimGroup {
        const types::SimulationResult* sample = nullptr;
        std::string key;
        std::vector<MissionGroup> missions;
    };

    const auto simKey = [](const types::SimulationResult& r) {
        std::ostringstream os;
        os << r.simulation_config.map_filename.string() << '|'
           << r.simulation_config.map_resolution.force_numerical_value_in(cm);
        return os.str();
    };
    const auto missionKey = [](const types::SimulationResult& r) {
        const types::MappingBounds& b = r.mission_config.mission_bounds;
        std::ostringstream os;
        os << r.mission_config.max_steps << '|'
           << r.mission_config.gps_resolution.force_numerical_value_in(cm) << '|'
           << b.min_x.force_numerical_value_in(cm) << ',' << b.max_x.force_numerical_value_in(cm) << ','
           << b.min_y.force_numerical_value_in(cm) << ',' << b.max_y.force_numerical_value_in(cm) << ','
           << b.min_height.force_numerical_value_in(cm) << ',' << b.max_height.force_numerical_value_in(cm);
        return os.str();
    };

    std::vector<SimGroup> groups;
    for (const types::SimulationResult& r : report.runs) {
        const std::string sk = simKey(r);
        SimGroup* sim_group = nullptr;
        for (SimGroup& g : groups) {
            if (g.key == sk) {
                sim_group = &g;
                break;
            }
        }
        if (sim_group == nullptr) {
            groups.push_back(SimGroup{&r, sk, {}});
            sim_group = &groups.back();
        }

        const std::string mk = missionKey(r);
        MissionGroup* mission_group = nullptr;
        for (MissionGroup& m : sim_group->missions) {
            if (m.key == mk) {
                mission_group = &m;
                break;
            }
        }
        if (mission_group == nullptr) {
            sim_group->missions.push_back(MissionGroup{&r, mk, {}});
            mission_group = &sim_group->missions.back();
        }
        mission_group->runs.push_back(&r);
    }

    // Summary statistics over scored runs.
    std::size_t scored_runs = 0;
    double sum = 0.0;
    double min_score = 0.0;
    double max_score = 0.0;
    bool any_scored = false;
    for (const types::SimulationResult& r : report.runs) {
        if (!isScored(r)) {
            continue;
        }
        ++scored_runs;
        sum += r.mission_score;
        if (!any_scored) {
            min_score = r.mission_score;
            max_score = r.mission_score;
            any_scored = true;
        } else {
            min_score = (r.mission_score < min_score) ? r.mission_score : min_score;
            max_score = (r.mission_score > max_score) ? r.mission_score : max_score;
        }
    }
    const std::size_t total_runs = report.runs.size();
    const std::size_t error_runs = total_runs - scored_runs;
    const double average_score = (scored_runs > 0) ? (sum / static_cast<double>(scored_runs)) : 0.0;

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "score_report" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "composition_file" << YAML::Value << composition_file.string();
    out << YAML::Key << "generated_at_utc" << YAML::Value << report.generated_at_utc;
    out << YAML::Key << "metric" << YAML::Value << report.metric;
    out << YAML::Key << "score_range" << YAML::Value << YAML::BeginMap
        << YAML::Key << "min" << YAML::Value << std::get<0>(report.score_range)
        << YAML::Key << "max" << YAML::Value << std::get<1>(report.score_range)
        << YAML::EndMap;
    out << YAML::Key << "error_score" << YAML::Value << report.error_score;

    out << YAML::Key << "summary" << YAML::Value << YAML::BeginMap
        << YAML::Key << "total_runs" << YAML::Value << total_runs
        << YAML::Key << "scored_runs" << YAML::Value << scored_runs
        << YAML::Key << "error_runs" << YAML::Value << error_runs
        << YAML::Key << "average_score" << YAML::Value << average_score
        << YAML::Key << "min_score" << YAML::Value << min_score
        << YAML::Key << "max_score" << YAML::Value << max_score
        << YAML::EndMap;

    out << YAML::Key << "simulations" << YAML::Value << YAML::BeginSeq;
    for (const SimGroup& sg : groups) {
        out << YAML::BeginMap;
        out << YAML::Key << "simulation_map" << YAML::Value << sg.sample->simulation_config.map_filename.string();
        out << YAML::Key << "map_resolution_cm" << YAML::Value
            << sg.sample->simulation_config.map_resolution.force_numerical_value_in(cm);
        out << YAML::Key << "missions" << YAML::Value << YAML::BeginSeq;
        for (const MissionGroup& mg : sg.missions) {
            const types::MissionConfigData& mc = mg.sample->mission_config;
            const types::MapConfig& oc = mg.sample->output_map_config;
            out << YAML::BeginMap;
            out << YAML::Key << "max_steps" << YAML::Value << mc.max_steps;
            out << YAML::Key << "resolution_cm" << YAML::Value << oc.resolution.force_numerical_value_in(cm);
            out << YAML::Key << "resolution_request_status" << YAML::Value
                << resolutionStatusString(mg.sample->resolution_request_status);
            out << YAML::Key << "runs" << YAML::Value << YAML::BeginSeq;
            for (const types::SimulationResult* rp : mg.runs) {
                const types::SimulationResult& r = *rp;
                const types::MissionRunResult mission_result =
                    r.mission_results.empty() ? types::MissionRunResult{} : r.mission_results.front();
                out << YAML::BeginMap;
                out << YAML::Key << "output_map" << YAML::Value << r.output_map_file.string();
                out << YAML::Key << "status" << YAML::Value << statusString(mission_result.status);
                out << YAML::Key << "steps" << YAML::Value << mission_result.steps;
                out << YAML::Key << "score" << YAML::Value << r.mission_score;
                if (!mission_result.errors.empty()) {
                    out << YAML::Key << "error_ref" << YAML::Value << YAML::BeginMap
                        << YAML::Key << "code" << YAML::Value << mission_result.errors.front().code
                        << YAML::Key << "message" << YAML::Value << mission_result.errors.front().message
                        << YAML::EndMap;
                }
                out << YAML::EndMap;
            }
            out << YAML::EndSeq; // runs
            out << YAML::EndMap;  // mission
        }
        out << YAML::EndSeq; // missions
        out << YAML::EndMap;  // simulation
    }
    out << YAML::EndSeq; // simulations

    out << YAML::EndMap; // score_report
    out << YAML::EndMap; // root

    std::error_code ec;
    if (output_file.has_parent_path()) {
        std::filesystem::create_directories(output_file.parent_path(), ec);
    }
    std::ofstream file(output_file);
    if (file) {
        file << out.c_str() << "\n";
    }
}

} // namespace drone_mapper
