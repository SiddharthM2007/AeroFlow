#include "aeroflow/metadata.hpp"

#include <cstdint>
#include <span>

#include "aeroflow/base64.hpp"
#include "aeroflow/box.hpp"

namespace aeroflow {

using nlohmann::json;

Grid3D build_grid(const GridConfig& config) {
  Grid3D grid(config.nx, config.ny, config.nz, config.lx, config.ly, config.lz);
  if (config.test_box) mark_solid_box(grid, *config.test_box);
  return grid;
}

json grid_metadata(const Grid3D& grid, const GridConfig& config, bool include_cells) {
  const std::size_t solid = grid.count(CellType::Solid);

  json out;
  out["engine"] = {{"name", "aeroflow-grid"}, {"version", AEROFLOW_VERSION}};
  out["grid"] = {
      {"resolution", {{"nx", grid.nx()}, {"ny", grid.ny()}, {"nz", grid.nz()}}},
      {"domain", {{"lx", grid.lx()}, {"ly", grid.ly()}, {"lz", grid.lz()}}},
      {"spacing", {{"dx", grid.dx()}, {"dy", grid.dy()}, {"dz", grid.dz()}}},
      {"origin", {0.0, 0.0, 0.0}},
      {"cell_count", grid.cell_count()},
      {"cell_volume", grid.cell_volume()},
      {"index_order", "i + nx*(j + ny*k)"},
      {"cell_center", "((i+0.5)*dx, (j+0.5)*dy, (k+0.5)*dz)"},
  };

  json classification = {
      {"fluid_count", grid.cell_count() - solid},
      {"solid_count", solid},
      {"solid_rule", "cell center in half-open test box [min, max)"},
      {"values", {{"fluid", static_cast<int>(CellType::Fluid)},
                  {"solid", static_cast<int>(CellType::Solid)}}},
  };
  if (include_cells) {
    static_assert(sizeof(CellType) == 1);
    const auto cells = grid.cells();
    const std::span<const std::uint8_t> bytes(reinterpret_cast<const std::uint8_t*>(cells.data()),
                                              cells.size());
    classification["encoding"] = "base64-uint8";
    classification["data"] = base64_encode(bytes);
  }
  out["classification"] = std::move(classification);

  json warnings = json::array();
  if (config.test_box) {
    const Aabb& b = *config.test_box;
    out["test_box"] = {{"min", {b.min.x, b.min.y, b.min.z}}, {"max", {b.max.x, b.max.y, b.max.z}}};
    if (solid == 0) {
      warnings.push_back("test box contains no cell centers at this resolution; no cells are solid");
    }
  } else {
    out["test_box"] = nullptr;
  }
  out["warnings"] = std::move(warnings);
  return out;
}

}  // namespace aeroflow
