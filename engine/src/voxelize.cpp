#include "aeroflow/voxelize.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "aeroflow/bvh.hpp"
#include "aeroflow/ray.hpp"

namespace aeroflow {

namespace {

using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

Vec3 normalized(Vec3 v) {
  const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  return {v.x / n, v.y / n, v.z / n};
}

// Fixed, deterministic fallback directions. Every component is non-zero and the ratios are
// irrational-looking, so they are not parallel to axis-aligned faces or common CAD symmetries.
const std::array<Vec3, 6>& fallback_directions() {
  static const std::array<Vec3, 6> dirs = {
      normalized({1.0, 0.4142135623730951, 0.2320508075688772}),
      normalized({-0.3183098861837907, 1.0, 0.5772156649015329}),
      normalized({0.2718281828459045, -0.6931471805599453, 1.0}),
      normalized({-1.0, -0.1414213562373095, 0.3819660112501051}),
      normalized({0.6180339887498949, 0.2078795763507619, -1.0}),
      normalized({0.1234567890123457, -1.0, -0.4655712318767680}),
  };
  return dirs;
}

double diagonal(const Vec3& lo, const Vec3& hi) {
  return std::hypot(hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
}

enum class RayVerdict { Inside, Outside, OnSurface, Ambiguous };

}  // namespace

VoxelizeStats voxelize_mesh(Grid3D& grid, const TriangleMesh& mesh, const VoxelizeOptions& options) {
  VoxelizeStats stats;
  stats.triangles = mesh.triangles.size();
  stats.cells_classified = grid.cell_count();
  grid.fill(CellType::Fluid);
  if (mesh.triangles.empty()) return stats;

  const Aabb mb = mesh_bounds(mesh);
  // Length tolerance relative to the problem size, never an absolute constant.
  const double scale = std::max(diagonal({0, 0, 0}, {grid.lx(), grid.ly(), grid.lz()}), diagonal(mb.min, mb.max));
  const double tol = 1e-10 * scale;
  stats.tolerance_m = tol;
  RayTolerances ray_tol;
  ray_tol.length = tol;

  const auto build_start = Clock::now();
  std::optional<Bvh> bvh;
  if (options.use_bvh) {
    bvh.emplace(mesh, options.bvh_leaf_size, tol);
    stats.bvh_nodes = bvh->nodes().size();
    stats.bvh_depth = bvh->depth();
  }
  stats.bvh_build_ms = ms_since(build_start);

  // Visits candidate triangles for a ray (BVH or brute force).
  auto for_candidates = [&](const Vec3& o, const Vec3& d, double t_min, auto&& visit) {
    ++stats.rays_cast;
    if (bvh) {
      stats.ray_box_tests += bvh->traverse(o, d, t_min, visit);
    } else {
      for (std::uint32_t n = 0; n < mesh.triangles.size(); ++n) visit(n);
    }
  };

  // Re-cast from a cell center along the fallback directions.
  auto classify_point = [&](const Vec3& p) -> RayVerdict {
    for (const Vec3& dir : fallback_directions()) {
      ++stats.fallback_rays;
      std::size_t crossings = 0;
      bool ambiguous = false;
      bool on_surface = false;
      for_candidates(p, dir, -tol, [&](std::uint32_t n) {
        if (on_surface) return;
        ++stats.ray_triangle_tests;
        const RayHit h = intersect_ray_triangle(p, dir, mesh.triangles[n], ray_tol);
        if (h.kind == HitKind::Miss) return;
        if (h.kind == HitKind::Ambiguous && std::isnan(h.t)) {
          ambiguous = true;  // ray lies in a triangle's plane
        } else if (std::abs(h.t) <= tol) {
          on_surface = true;
        } else if (h.t > tol) {
          if (h.kind == HitKind::Ambiguous) ambiguous = true;
          else ++crossings;
        }
      });
      if (on_surface) return RayVerdict::OnSurface;
      if (!ambiguous) return crossings % 2 == 1 ? RayVerdict::Inside : RayVerdict::Outside;
    }
    return RayVerdict::Ambiguous;
  };

  const auto classify_start = Clock::now();
  const Vec3 dir_x{1.0, 0.0, 0.0};
  const double origin_x = mb.min.x - 0.5 * (mb.max.x - mb.min.x) - 1e3 * tol;
  std::vector<double> clean;
  auto cells = grid.cells();

  for (std::size_t k = 0; k < grid.nz(); ++k) {
    const double z = grid.cell_center_z(k);
    for (std::size_t j = 0; j < grid.ny(); ++j) {
      ++stats.rows_total;
      const double y = grid.cell_center_y(j);
      if (y < mb.min.y - tol || y > mb.max.y + tol || z < mb.min.z - tol || z > mb.max.z + tol) {
        ++stats.rows_skipped;
        continue;
      }

      // Gather crossings of the row line with the mesh.
      clean.clear();
      double ambiguous_reach = -std::numeric_limits<double>::infinity();  // rightmost ambiguous x
      const Vec3 origin{origin_x, y, z};
      for_candidates(origin, dir_x, 0.0, [&](std::uint32_t n) {
        ++stats.ray_triangle_tests;
        const Triangle& tri = mesh.triangles[n];
        const RayHit h = intersect_ray_triangle(origin, dir_x, tri, ray_tol);
        if (h.kind == HitKind::Miss) return;
        if (h.kind == HitKind::Hit) {
          clean.push_back(origin_x + h.t);
          return;
        }
        ++stats.ambiguous_crossings;
        // In-plane rays have no single crossing point: the whole triangle's x-range is uncertain.
        const double reach = std::isnan(h.t) ? std::max({tri.a.x, tri.b.x, tri.c.x}) : origin_x + h.t;
        ambiguous_reach = std::max(ambiguous_reach, reach);
      });
      if (clean.empty() && ambiguous_reach == -std::numeric_limits<double>::infinity()) continue;
      std::sort(clean.begin(), clean.end());

      for (std::size_t i = 0; i < grid.nx(); ++i) {
        const double x = grid.cell_center_x(i);
        if (x < mb.min.x - tol || x > mb.max.x + tol) continue;  // outside mesh bounds: Fluid
        CellType type;
        if (x <= ambiguous_reach + tol) {
          // Parity to the right of x depends on an ambiguous crossing: resolve per cell.
          ++stats.fallback_cells;
          switch (classify_point({x, y, z})) {
            case RayVerdict::Inside: type = CellType::Solid; break;
            case RayVerdict::Outside: type = CellType::Fluid; break;
            case RayVerdict::OnSurface:
              ++stats.surface_cells;
              type = CellType::Solid;
              break;
            case RayVerdict::Ambiguous:
            default:
              ++stats.unresolved_cells;
              type = CellType::Solid;
              break;
          }
        } else {
          const auto right = std::upper_bound(clean.begin(), clean.end(), x);
          const bool near_right = right != clean.end() && *right - x <= tol;
          const bool near_left = right != clean.begin() && x - *(right - 1) <= tol;
          if (near_right || near_left) {
            ++stats.surface_cells;
            type = CellType::Solid;
          } else {
            const auto crossings = static_cast<std::size_t>(clean.end() - right);
            type = crossings % 2 == 1 ? CellType::Solid : CellType::Fluid;
          }
        }
        cells[grid.index(i, j, k)] = type;
      }
    }
  }
  stats.classify_ms = ms_since(classify_start);
  stats.solid_cells = grid.count(CellType::Solid);
  return stats;
}

}  // namespace aeroflow
