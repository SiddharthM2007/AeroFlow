// aeroflow-grid: reads a grid configuration (JSON) and writes grid metadata (JSON).
//
// Exit codes: 0 success, 1 usage error, 2 invalid configuration, 3 I/O or internal error.
// On failure a JSON object {"error": {"code": ..., "message": ...}} is written to stderr.

#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

#include "aeroflow/config.hpp"
#include "aeroflow/field_export.hpp"
#include "aeroflow/simulation.hpp"
#include "aeroflow/metadata.hpp"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitUsage = 1;
constexpr int kExitConfig = 2;
constexpr int kExitIo = 3;

constexpr std::string_view kUsage =
    "Usage: aeroflow-grid <config.json | -> [--output FILE] [--no-cells] [--pretty]\n"
    "\n"
    "Reads a grid configuration (use '-' for stdin) and writes grid metadata as JSON.\n"
    "\n"
    "Options:\n"
    "  --output FILE  Write metadata to FILE instead of stdout\n"
    "  --no-cells     Omit the per-cell classification array\n"
    "  --pretty       Indent the JSON output\n"
    "  --help         Show this message\n"
    "  --version      Show the engine version\n";

int report_error(int code, std::string_view kind, const std::string& message) {
  const nlohmann::json err = {{"error", {{"code", kind}, {"message", message}}}};
  std::cerr << err.dump() << '\n';
  return code;
}

struct Options {
  std::string config_path;
  std::string output_path;
  bool include_cells = true;
  bool pretty = false;
};

}  // namespace

int main(int argc, char** argv) {
  Options opts;
  for (int n = 1; n < argc; ++n) {
    const std::string_view arg = argv[n];
    if (arg == "--help" || arg == "-h") {
      std::cout << kUsage;
      return kExitOk;
    }
    if (arg == "--version") {
      std::cout << "aeroflow-grid " << AEROFLOW_VERSION << '\n';
      return kExitOk;
    }
    if (arg == "--no-cells") {
      opts.include_cells = false;
    } else if (arg == "--pretty") {
      opts.pretty = true;
    } else if (arg == "--output") {
      if (n + 1 >= argc) return report_error(kExitUsage, "usage", "--output requires a file path");
      opts.output_path = argv[++n];
    } else if (arg.size() > 1 && arg.starts_with("-")) {
      return report_error(kExitUsage, "usage", "unknown option: " + std::string(arg));
    } else if (opts.config_path.empty()) {
      opts.config_path = arg;
    } else {
      return report_error(kExitUsage, "usage", "unexpected argument: " + std::string(arg));
    }
  }
  if (opts.config_path.empty()) {
    std::cerr << kUsage;
    return report_error(kExitUsage, "usage", "missing config path");
  }

  std::string text;
  if (opts.config_path == "-") {
    text.assign(std::istreambuf_iterator<char>(std::cin), std::istreambuf_iterator<char>());
  } else {
    std::ifstream in(opts.config_path, std::ios::binary);
    if (!in) return report_error(kExitIo, "io", "cannot open config file: " + opts.config_path);
    std::ostringstream buffer;
    buffer << in.rdbuf();
    text = buffer.str();
  }

  try {
    const aeroflow::GridConfig config = aeroflow::parse_config_text(text);
    const aeroflow::BuiltGrid built = aeroflow::build(config);
    nlohmann::json meta =
        aeroflow::grid_metadata(built.grid, config, opts.include_cells, built.mesh ? &*built.mesh : nullptr);
    meta["fields"] = nlohmann::json::array();
    meta["projection"] = nullptr;
    meta["simulation"] = nullptr;
    if (config.fields) {
      // Progress lines go to stderr as JSON ({"progress": {...}}); the final error, if any, is
      // always the last stderr line.
      const auto progress = [](const nlohmann::json& p) {
        std::cerr << nlohmann::json{{"progress", p}}.dump() << std::endl;
      };
      auto exported = aeroflow::export_fields(built.grid, config.fields->requested, config.fields->output_dir,
                                              config.projection.value_or(aeroflow::ProjectionConfig{}),
                                              config.simulation.value_or(aeroflow::SimulationConfig{}), progress);
      meta["fields"] = std::move(exported.fields);
      meta["projection"] = std::move(exported.projection);
      meta["simulation"] = std::move(exported.simulation);
    }
    const std::string body = meta.dump(opts.pretty ? 2 : -1);

    if (opts.output_path.empty()) {
      std::cout << body << '\n';
    } else {
      std::ofstream out(opts.output_path, std::ios::binary);
      if (!out) return report_error(kExitIo, "io", "cannot write output file: " + opts.output_path);
      out << body << '\n';
    }
  } catch (const aeroflow::SimulationError& e) {
    const bool user_error = e.code() != "non_finite";
    return report_error(user_error ? kExitConfig : kExitIo,
                        e.code() == "invalid_settings" ? "invalid_simulation_settings" : e.code(), e.what());
  } catch (const aeroflow::ProjectionError& e) {
    const bool user_error = e.code() == "invalid_settings" || e.code() == "incompatible_component";
    return report_error(user_error ? kExitConfig : kExitIo,
                        e.code() == "invalid_settings" ? "invalid_projection_settings" : e.code(), e.what());
  } catch (const aeroflow::BoundaryError& e) {
    return report_error(kExitConfig, "invalid_boundaries", e.what());
  } catch (const aeroflow::FieldError& e) {
    return report_error(kExitIo, "field_export", e.what());
  } catch (const aeroflow::MeshError& e) {
    return report_error(kExitConfig, "invalid_mesh", e.what());
  } catch (const aeroflow::ConfigError& e) {
    return report_error(kExitConfig, "invalid_config", e.what());
  } catch (const std::invalid_argument& e) {
    return report_error(kExitConfig, "invalid_config", e.what());
  } catch (const std::exception& e) {
    return report_error(kExitIo, "internal", e.what());
  }
  return kExitOk;
}
