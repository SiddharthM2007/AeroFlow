#pragma once

#include <nlohmann/json.hpp>

#include <optional>

#include "aeroflow/config.hpp"
#include "aeroflow/grid3d.hpp"
#include "aeroflow/mesh.hpp"
#include "aeroflow/voxelize.hpp"

namespace aeroflow {

struct MeshVoxelization {
  MeshReport report;
  VoxelizeStats stats;
  double signed_volume = 0.0;
};

struct BuiltGrid {
  Grid3D grid;
  std::optional<MeshVoxelization> mesh;
};

// Builds the grid described by `config`: marks the procedural test box solid, or reads,
// validates and voxelizes the mesh. Throws MeshError for unreadable or invalid meshes.
BuiltGrid build(const GridConfig& config);

// Procedural-only convenience wrapper around build().
Grid3D build_grid(const GridConfig& config);

// JSON metadata describing `grid`. When `include_cells` is true, the per-cell
// classification is embedded as base64-encoded uint8 values in linear index order.
nlohmann::json grid_metadata(const Grid3D& grid, const GridConfig& config, bool include_cells,
                             const MeshVoxelization* mesh = nullptr);

}  // namespace aeroflow
