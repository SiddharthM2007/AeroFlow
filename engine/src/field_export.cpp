#include "aeroflow/field_export.hpp"

#include <filesystem>
#include <stdexcept>

#include "aeroflow/divergence.hpp"
#include "aeroflow/mac_grid.hpp"

namespace aeroflow {

std::vector<std::string> known_field_ids() { return {kDivergenceDiagnostic}; }

ScalarField divergence_diagnostic(const Grid3D& mask) {
  MacGrid mac(mask);
  const FaceTypes faces = classify_faces(mask);
  mac.set_uniform_velocity({kDiagnosticInflowSpeed, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  const DivergenceField div = compute_divergence(mac, mask);

  ScalarField f;
  f.name = kDivergenceDiagnostic;
  f.quantity = "divergence";
  f.long_name = "Velocity divergence of the initialized field";
  f.units = "1/s";
  f.is_signed = true;
  f.location = FieldLocation::Cell;
  f.shape = field_shape(FieldLocation::Cell, mask.nx(), mask.ny(), mask.nz());
  const auto data = div.values.data();
  f.values.assign(data.begin(), data.end());
  f.valid.resize(mask.cell_count());
  const auto cells = mask.cells();
  for (std::size_t n = 0; n < cells.size(); ++n) f.valid[n] = cells[n] == CellType::Fluid ? 1 : 0;
  f.frame = {{"index", 0}, {"time_s", nullptr}, {"label", "initialized"}};
  f.mask = {{"source", "grid_classification"}, {"valid_rule", "fluid cells"}};
  f.provenance = {
      {"kind", "diagnostic"},
      {"solved", false},
      {"description",
       "Divergence of a prescribed initial field: uniform +x 5 m/s on active faces (tunnel walls "
       "included), 0 on faces blocked by solids. Not a CFD solution."},
      {"initial_velocity", {kDiagnosticInflowSpeed, 0.0, 0.0}},
      {"blocked_faces", "normal velocity 0"},
  };
  return f;
}

nlohmann::json export_fields(const Grid3D& mask, const std::vector<std::string>& requested,
                             const std::string& output_dir) {
  nlohmann::json summaries = nlohmann::json::array();
  if (requested.empty()) return summaries;
  if (!std::filesystem::is_directory(output_dir)) {
    throw FieldError("field output directory does not exist: " + output_dir);
  }
  const MacGrid geometry(mask);  // spacing/extents for headers (same convention as Grid3D)
  for (const auto& id : requested) {
    if (id != kDivergenceDiagnostic) throw std::invalid_argument("unknown field id: " + id);
    const ScalarField field = divergence_diagnostic(mask);
    const std::string file = field.name + ".f0.affield";
    const auto header = write_field_file((std::filesystem::path(output_dir) / file).string(), field, geometry);
    summaries.push_back({
        {"name", header["name"]},
        {"file", file},
        {"quantity", header["quantity"]},
        {"location", header["location"]},
        {"shape", header["shape"]},
        {"units", header["units"]},
        {"frame", header["frame"]},
        {"stats", header["stats"]},
        {"provenance", header["provenance"]},
    });
  }
  return summaries;
}

}  // namespace aeroflow
