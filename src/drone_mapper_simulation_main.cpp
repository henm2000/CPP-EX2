#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationReportWriter.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/YamlConfigLoader.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>

// ./drone_mapper_simulation [<simulation.yaml>] [<output_path>]
//   - missing first argument  -> "simulation.yaml" in the current directory
//   - bare filename / relative -> resolved under the current directory
//   - absolute path           -> used as-is
// The program never calls exit(); it always returns from main.
int main(int argc, char** argv) {
    namespace fs = std::filesystem;

    fs::path composition_file = (argc >= 2) ? fs::path{argv[1]} : fs::path{"simulation.yaml"};
    fs::path output_path = (argc >= 3) ? fs::path{argv[2]} : fs::current_path();

    std::error_code ec;
    const fs::path cwd = fs::current_path(ec);
    if (composition_file.is_relative()) {
        composition_file = cwd / composition_file;
    }
    if (output_path.is_relative()) {
        output_path = cwd / output_path;
    }

    try {
        const drone_mapper::types::SimulationCompositionData composition =
            drone_mapper::YamlConfigLoader::loadComposition(composition_file);

        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        drone_mapper::SimulationManager simulation{std::move(run_factory)};

        const drone_mapper::types::SimulationManagerReport report =
            simulation.run(composition, output_path);

        const fs::path report_file = output_path / "simulation_output.yaml";
        drone_mapper::SimulationReportWriter::write(report, composition.composition_file, report_file);

        std::cout << "Wrote " << report_file.string()
                  << " (" << report.runs.size() << " run(s)).\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
