#pragma once

#include <cstddef>

#include "aeroflow/grid3d.hpp"
#include "aeroflow/mesh.hpp"

namespace aeroflow {

struct VoxelizeOptions {
  std::size_t bvh_leaf_size = 4;
  // false: test every triangle for every ray (brute-force baseline, for verification only).
  bool use_bvh = true;
};

struct VoxelizeStats {
  std::size_t triangles = 0;
  std::size_t cells_classified = 0;
  std::size_t solid_cells = 0;
  std::size_t rows_total = 0;
  std::size_t rows_skipped = 0;  // (y, z) outside the mesh bounds: no ray cast
  std::size_t rays_cast = 0;     // row rays + fallback rays
  std::size_t ray_triangle_tests = 0;
  std::size_t ray_box_tests = 0;
  std::size_t ambiguous_crossings = 0;
  std::size_t fallback_cells = 0;  // cells resolved by re-casting from the cell center
  std::size_t fallback_rays = 0;
  std::size_t surface_cells = 0;     // centers within tolerance of the surface -> Solid
  std::size_t unresolved_cells = 0;  // every fallback direction ambiguous -> Solid (policy)
  std::size_t bvh_nodes = 0;
  std::size_t bvh_depth = 0;
  double tolerance_m = 0.0;
  double bvh_build_ms = 0.0;
  double classify_ms = 0.0;
};

// Classifies every cell of `grid` as Solid or Fluid from a closed triangle mesh in world
// coordinates (meters), using the cell-center rule:
//
//   cell (i,j,k) is Solid  <=>  center p_ijk = ((i+.5)dx, (j+.5)dy, (k+.5)dz) is inside the mesh.
//
// Inside/outside is decided by odd/even parity of crossings along a +x ray. One ray is cast per
// grid row (j,k); the parity for cell i is the number of crossings with x > x_i, which is the
// same as casting a separate +x ray from each center.
//
// Robustness policy (deterministic, no random jitter):
//   * Crossings within the barycentric tolerance of an edge/vertex, or rays lying in a
//     triangle's plane, are Ambiguous and are never counted.
//   * A cell whose parity depends on an ambiguous crossing is re-cast from its center along a
//     fixed sequence of non-axis-aligned directions; the first fully unambiguous ray decides.
//   * A center within `tolerance_m` of the surface is on the surface and is classified Solid.
//   * If every fallback direction is ambiguous, the cell is classified Solid (same policy).
//
// The mesh should have been checked with validate_mesh(). Every cell is overwritten.
VoxelizeStats voxelize_mesh(Grid3D& grid, const TriangleMesh& mesh, const VoxelizeOptions& options = {});

}  // namespace aeroflow
