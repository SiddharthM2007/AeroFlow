#include "aeroflow/metadata.hpp"

#include <cmath>
#include <cstdint>
#include <span>

#include "aeroflow/base64.hpp"
#include "aeroflow/box.hpp"

namespace aeroflow {

using nlohmann::json;

BuiltGrid build(const GridConfig& config) {
  BuiltGrid built{Grid3D(config.nx, config.ny, config.nz, config.lx, config.ly, config.lz), std::nullopt};
  if (config.test_box) mark_solid_box(built.grid, *config.test_box);
  if (config.mesh) {
    TriangleMesh mesh = read_triangle_file(config.mesh->triangles_file);
    MeshVoxelization result;
    result.report = validate_mesh(mesh);
    result.signed_volume = signed_volume(mesh);
    VoxelizeOptions options;
    options.bvh_leaf_size = config.mesh->bvh_leaf_size;
    result.stats = voxelize_mesh(built.grid, mesh, options);
    built.mesh = std::move(result);
  }
  return built;
}

Grid3D build_grid(const GridConfig& config) { return std::move(build(config).grid); }

namespace {

json point(const Vec3& p) { return {p.x, p.y, p.z}; }

json mesh_metadata(const MeshVoxelization& m, const Grid3D& grid, std::size_t solid, json& warnings) {
  const MeshReport& r = m.report;
  const bool inside = r.bounds.min.x >= 0.0 && r.bounds.min.y >= 0.0 && r.bounds.min.z >= 0.0 &&
                      r.bounds.max.x <= grid.lx() && r.bounds.max.y <= grid.ly() &&
                      r.bounds.max.z <= grid.lz();
  const double voxel_volume = static_cast<double>(solid) * grid.cell_volume();

  json out = {
      {"input_triangles", r.input_triangles},
      {"triangles", r.triangles},
      {"degenerate_removed", r.degenerate_removed},
      {"unique_vertices", r.unique_vertices},
      {"watertight", true},
      {"consistently_oriented", r.consistently_oriented},
      {"bounds", {{"min", point(r.bounds.min)}, {"max", point(r.bounds.max)}}},
      {"inside_domain", inside},
      {"voxel_volume", voxel_volume},
      {"volume", nullptr},
      {"orientation", nullptr},
      {"volume_relative_error", nullptr},
  };
  if (r.consistently_oriented) {
    const double volume = std::abs(m.signed_volume);
    out["volume"] = volume;
    out["orientation"] = m.signed_volume >= 0.0 ? "outward" : "inward";
    if (inside && volume > 0.0) out["volume_relative_error"] = std::abs(voxel_volume - volume) / volume;
  }
  for (const auto& w : r.warnings) warnings.push_back(w);
  if (!inside) {
    warnings.push_back("mesh extends outside the tunnel; only the part inside the domain is classified");
  }
  if (m.stats.unresolved_cells > 0) {
    warnings.push_back(std::to_string(m.stats.unresolved_cells) +
                       " cell(s) had no unambiguous ray in any fallback direction and were classified solid");
  }
  return out;
}

json stats_metadata(const VoxelizeStats& s) {
  return {
      {"triangles", s.triangles},
      {"cells_classified", s.cells_classified},
      {"rows_total", s.rows_total},
      {"rows_skipped", s.rows_skipped},
      {"rays_cast", s.rays_cast},
      {"ray_triangle_tests", s.ray_triangle_tests},
      {"ray_box_tests", s.ray_box_tests},
      {"ambiguous_crossings", s.ambiguous_crossings},
      {"fallback_cells", s.fallback_cells},
      {"fallback_rays", s.fallback_rays},
      {"surface_cells", s.surface_cells},
      {"unresolved_cells", s.unresolved_cells},
      {"bvh_nodes", s.bvh_nodes},
      {"bvh_depth", s.bvh_depth},
      {"tolerance_m", s.tolerance_m},
      {"bvh_build_ms", s.bvh_build_ms},
      {"classify_ms", s.classify_ms},
  };
}

}  // namespace

json grid_metadata(const Grid3D& grid, const GridConfig& config, bool include_cells,
                   const MeshVoxelization* mesh) {
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
      {"method", "cell_center"},
      {"solid_rule", mesh ? "cell center inside the closed triangle mesh (+x ray parity; centers on the surface are solid)"
                          : "cell center in half-open test box [min, max)"},
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
  out["geometry_source"] = mesh ? "mesh" : (config.test_box ? "procedural_box" : "none");
  if (mesh) {
    out["mesh"] = mesh_metadata(*mesh, grid, solid, warnings);
    out["voxelization"] = stats_metadata(mesh->stats);
    if (solid == 0) warnings.push_back("no cell centers fall inside the mesh at this resolution; no cells are solid");
  }
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
