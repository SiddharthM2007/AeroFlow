#pragma once

#include <nlohmann/json.hpp>

#include "aeroflow/config.hpp"
#include "aeroflow/grid3d.hpp"

namespace aeroflow {

// Builds the grid described by `config` (marking the optional test box solid).
Grid3D build_grid(const GridConfig& config);

// JSON metadata describing `grid`. When `include_cells` is true, the per-cell
// classification is embedded as base64-encoded uint8 values in linear index order.
nlohmann::json grid_metadata(const Grid3D& grid, const GridConfig& config, bool include_cells);

}  // namespace aeroflow
