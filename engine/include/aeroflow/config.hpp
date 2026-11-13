#pragma once

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "aeroflow/pressure.hpp"
#include "aeroflow/box.hpp"

namespace aeroflow {

// Raised for any malformed or out-of-range configuration. The message names the
// offending field (e.g. "resolution.nx: must be an integer >= 1").
class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Imported mesh geometry: world-space triangles (see mesh.hpp for the file format).
struct MeshSource {
  std::string triangles_file;
  std::size_t bvh_leaf_size = 4;
};

// Numerical fields to export as AFFIELD files (see field_io.hpp / field_export.hpp).
struct FieldsRequest {
  std::string output_dir;
  std::vector<std::string> requested;
};

// Pressure projection settings (Milestone 5); validated here so CLI errors name the field.
struct ProjectionConfig {
  double inlet_velocity = 5.0;            // m/s, finite, >= 0
  double density = 1.225;                 // kg/m^3, > 0
  std::optional<double> dt;               // s, > 0; default CFL 0.5 min(h) / U_in
  double tolerance = 1e-10;               // relative residual, (0, 1)
  std::size_t max_iterations = 10000;     // 1 .. 1,000,000
  PressureSolverKind pressure_solver = PressureSolverKind::Auto;  // "auto" | "mgpcg" | "jacobi_pcg"
};

// Time-dependent Navier-Stokes run (Milestone 6); mirrors aeroflow::SimulationSettings.
struct SimulationConfig {
  double inlet_velocity = 5.0;      // m/s, >= 0
  double density = 1.225;           // kg/m^3, > 0
  double viscosity = 1.5e-5;        // m^2/s, >= 0
  double duration = 0.5;            // s, > 0
  double frame_interval = 0.05;     // s, > 0
  double cfl = 0.5;                 // (0, 1]
  std::optional<double> dt;         // s, > 0; upper limit, rejected if unstable at t = 0
  std::size_t max_steps = 2000;     // 1 .. 1,000,000
  double tolerance = 1e-8;          // per-step PCG relative residual, (0, 1)
  std::size_t max_iterations = 10000;
  std::string initial = "uniform";  // "uniform" | "rest"
  double max_wall_seconds = 900.0;  // (0, 86400]; hard stop keeping frames written so far
  PressureSolverKind pressure_solver = PressureSolverKind::Auto;  // "auto" | "mgpcg" | "jacobi_pcg"
};

struct GridConfig {
  std::size_t nx{0};
  std::size_t ny{0};
  std::size_t nz{0};
  double lx{0.0};
  double ly{0.0};
  double lz{0.0};
  // Geometry sources are mutually exclusive; at most one is set.
  std::optional<Aabb> test_box;
  std::optional<MeshSource> mesh;
  std::optional<FieldsRequest> fields;
  std::optional<ProjectionConfig> projection;
  std::optional<SimulationConfig> simulation;
};

// Expected shape:
// {
//   "domain":     {"lx": 4.0, "ly": 2.0, "lz": 2.0},
//   "resolution": {"nx": 64,  "ny": 32,  "nz": 32},
//   "test_box":   {"min": [x, y, z], "max": [x, y, z]},  // optional, procedural solid
//   "mesh":       {"triangles_file": "...", "bvh_leaf_size": 4}  // optional, imported mesh
// }
//   "fields":     {"output_dir": "...", "requested": ["divergence_diagnostic"]}  // optional
//   "projection": {"inlet_velocity": 5, "density": 1.225, "dt": null, "tolerance": 1e-10,
//                  "max_iterations": 10000}                                       // optional
//   "simulation": {"inlet_velocity", "density", "viscosity", "duration", "frame_interval", "cfl",
//                  "dt", "max_steps", "tolerance", "max_iterations", "initial", "max_wall_seconds"}
// "test_box" and "mesh" are mutually exclusive.
// Unknown keys are rejected so typos do not pass silently.
GridConfig parse_config(const nlohmann::json& root);

// Parses JSON text first; syntax errors are reported as ConfigError.
GridConfig parse_config_text(const std::string& text);

}  // namespace aeroflow
