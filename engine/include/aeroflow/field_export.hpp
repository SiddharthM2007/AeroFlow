#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "aeroflow/field_io.hpp"
#include "aeroflow/grid3d.hpp"

namespace aeroflow {

// Field ids the engine can export (grid-job opt-in). Unknown ids are configuration errors.
inline constexpr const char* kDivergenceDiagnostic = "divergence_diagnostic";
std::vector<std::string> known_field_ids();

// Prescribed velocity of the divergence diagnostic (m/s, tunnel frame).
inline constexpr double kDiagnosticInflowSpeed = 5.0;

// Initialized divergence diagnostic — NOT a solved flow:
//   * MAC grid on the solid mask; u = 5 m/s on active (Fluid and DomainBoundary) x-faces,
//     so the x-walls carry the same 5 m/s at inflow and outflow; v = w = 0 everywhere.
//   * Blocked faces (SolidBoundary, SolidInterior) are 0: no flow through obstacles.
//   * Divergence at fluid-cell centers; solid cells are invalid.
// Unobstructed regions are exactly divergence-free; fluid cells bordering an obstacle on an
// x-face show +-5/dx, measuring where the initialized field violates continuity.
ScalarField divergence_diagnostic(const Grid3D& mask);

// Exports each requested field to `<output_dir>/<name>.f0.affield` and returns one summary per
// field ({name, file, location, shape, units, frame, stats, provenance}).
// Throws ConfigError-compatible std::invalid_argument for unknown ids, FieldError on I/O.
nlohmann::json export_fields(const Grid3D& mask, const std::vector<std::string>& requested,
                             const std::string& output_dir);

}  // namespace aeroflow
