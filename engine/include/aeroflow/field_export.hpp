#pragma once

#include <functional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "aeroflow/config.hpp"
#include "aeroflow/field_io.hpp"
#include "aeroflow/grid3d.hpp"
#include "aeroflow/pressure.hpp"

namespace aeroflow {

// Field ids the engine can export (grid-job opt-in). Unknown ids are configuration errors.
inline constexpr const char* kDivergenceDiagnostic = "divergence_diagnostic";
// Milestone 5: produced by one pressure projection of the initialized field.
inline constexpr const char* kPressure = "pressure";
inline constexpr const char* kDivergenceBefore = "divergence_before";
inline constexpr const char* kDivergenceAfter = "divergence_after";
inline constexpr const char* kVelocityMagnitude = "velocity_magnitude";
// Milestone 6: time-indexed frames of a Navier-Stokes run (one file per field per frame).
inline constexpr const char* kSimPressure = "sim_pressure";
inline constexpr const char* kSimVelocityMagnitude = "sim_velocity_magnitude";
inline constexpr const char* kSimDivergence = "sim_divergence";
inline constexpr const char* kSimU = "sim_u";  // face_x
inline constexpr const char* kSimV = "sim_v";  // face_y
inline constexpr const char* kSimW = "sim_w";  // face_z
std::vector<std::string> known_field_ids();
bool is_projection_field(const std::string& id);
bool is_simulation_field(const std::string& id);

// Prescribed velocity of the divergence diagnostic (m/s, tunnel frame).
inline constexpr double kDiagnosticInflowSpeed = 5.0;

// Initialized divergence diagnostic — NOT a solved flow:
//   * MAC grid on the solid mask; u = U_in on active (Fluid and DomainBoundary) x-faces,
//     so the x-walls carry the same U_in at inflow and outflow; v = w = 0 everywhere.
//   * Blocked faces (SolidBoundary, SolidInterior) are 0: no flow through obstacles.
//   * Divergence at fluid-cell centers; solid cells are invalid.
// Unobstructed regions are exactly divergence-free; fluid cells bordering an obstacle on an
// x-face show +-U_in/dx, measuring where the initialized field violates continuity.
ScalarField divergence_diagnostic(const Grid3D& mask, double inlet_velocity = kDiagnosticInflowSpeed);

struct ProjectedFields {
  ProjectionResult result;
  ScalarField pressure;            // Pa
  ScalarField divergence_before;   // 1/s, of u* after velocity BCs
  ScalarField divergence_after;    // 1/s, recomputed independently after the correction
  ScalarField velocity_magnitude;  // m/s, cell-centered face averages
};

// One projection of the initialized field u* (uniform +x U_in on active faces, then velocity
// BCs). Throws ProjectionError / BoundaryError (e.g. blocked outlet, inlet pocket, no convergence).
ProjectedFields project_initialized_field(const Grid3D& mask, const ProjectionConfig& config);

// Projection statistics for the CLI / job result.
nlohmann::json projection_summary(const ProjectionResult& result, const ProjectionConfig& config);

struct FieldExport {
  nlohmann::json fields = nlohmann::json::array();  // one summary per exported field (and frame)
  nlohmann::json projection;                         // null unless a projection field was requested
  nlohmann::json simulation;                         // null unless a simulation field was requested
};

using ProgressCallback = std::function<void(const nlohmann::json&)>;

// Exports each requested field to `<output_dir>/<name>.f0.affield`. Projection fields share a
// single projection. Throws std::invalid_argument for unknown ids, FieldError on I/O, and
// ProjectionError / BoundaryError when the projection cannot be performed.
// Simulation fields run one Navier-Stokes simulation (simulation.hpp) writing every requested
// field at every output frame; `progress` receives throttled step updates.
FieldExport export_fields(const Grid3D& mask, const std::vector<std::string>& requested,
                          const std::string& output_dir, const ProjectionConfig& projection = {},
                          const SimulationConfig& simulation = {}, const ProgressCallback& progress = {});

}  // namespace aeroflow
