#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <numbers>
#include <optional>

#include "aeroflow/config.hpp"
#include "aeroflow/metadata.hpp"
#include "aeroflow/voxelize.hpp"
#include "mesh_fixtures.hpp"

using namespace aeroflow;
using fixtures::compose;
using fixtures::make_box;
using fixtures::make_octahedron;
using fixtures::make_sphere;
using fixtures::transformed;

namespace {

// Counts cells whose classification differs from inside(center); nullopt = don't care.
std::size_t mismatches(const Grid3D& g, const std::function<std::optional<bool>(const Vec3&)>& inside) {
  std::size_t bad = 0;
  for (std::size_t k = 0; k < g.nz(); ++k)
    for (std::size_t j = 0; j < g.ny(); ++j)
      for (std::size_t i = 0; i < g.nx(); ++i) {
        const auto expect = inside(g.cell_center(i, j, k));
        if (expect && (g.cell(i, j, k) == CellType::Solid) != *expect) ++bad;
      }
  return bad;
}

bool in_open_box(const Vec3& p, const Vec3& lo, const Vec3& hi) {
  return p.x > lo.x && p.x < hi.x && p.y > lo.y && p.y < hi.y && p.z > lo.z && p.z < hi.z;
}

std::string write_temp(const TriangleMesh& mesh, const char* name) {
  const auto path = (std::filesystem::temp_directory_path() / (std::string("aeroflow_") + name)).string();
  write_triangle_file(path, mesh);
  return path;
}

GridConfig mesh_config(std::size_t n, double L, const std::string& path) {
  GridConfig c;
  c.nx = c.ny = c.nz = n;
  c.lx = c.ly = c.lz = L;
  c.mesh = MeshSource{path, 4};
  return c;
}

}  // namespace

// --- cube -------------------------------------------------------------------------------------

TEST(VoxelizeCube, KnownCellsAndExactVolume) {
  // dx = 1/8: centers 0.0625 + 0.125 k; the cube [0.25, 0.75]^3 contains 4 centers per axis.
  Grid3D g(8, 8, 8, 1, 1, 1);
  const auto cube = make_box({0.25, 0.25, 0.25}, {0.75, 0.75, 0.75});
  const auto stats = voxelize_mesh(g, cube);
  EXPECT_EQ(stats.solid_cells, 64u);
  EXPECT_EQ(mismatches(g, [](const Vec3& p) { return in_open_box(p, {.25, .25, .25}, {.75, .75, .75}); }), 0u);
  EXPECT_EQ(g.cell(2, 2, 2), CellType::Solid);
  EXPECT_EQ(g.cell(5, 5, 5), CellType::Solid);
  EXPECT_EQ(g.cell(1, 2, 2), CellType::Fluid);
  EXPECT_EQ(g.cell(6, 5, 5), CellType::Fluid);
  // Voxel volume equals the analytical volume for this aligned case.
  EXPECT_DOUBLE_EQ(static_cast<double>(stats.solid_cells) * g.cell_volume(), signed_volume(cube));
  EXPECT_DOUBLE_EQ(signed_volume(cube), 0.125);
}

TEST(VoxelizeCube, AnisotropicGridAndOffsetBox) {
  Grid3D g(40, 13, 27, 4.0, 1.0, 2.0);
  const Vec3 lo{1.03, 0.21, 0.55}, hi{2.71, 0.66, 1.42};
  voxelize_mesh(g, make_box(lo, hi));
  EXPECT_EQ(mismatches(g, [&](const Vec3& p) { return in_open_box(p, lo, hi); }), 0u);
}

// --- shared edges / vertices / surface -------------------------------------------------------

TEST(VoxelizeEdgeCases, RaysThroughSharedTriangleEdges) {
  // dx = 1/4: rows with j == k pass exactly through the diagonal edge shared by the two
  // triangles of each x face. Naive counting would see 2 crossings per face (even -> fluid).
  Grid3D g(4, 4, 4, 1, 1, 1);
  const auto stats = voxelize_mesh(g, make_box({0.25, 0.25, 0.25}, {0.75, 0.75, 0.75}));
  EXPECT_GT(stats.ambiguous_crossings, 0u);
  EXPECT_GT(stats.fallback_cells, 0u);
  EXPECT_EQ(stats.unresolved_cells, 0u);
  EXPECT_EQ(stats.solid_cells, 8u);
  EXPECT_EQ(mismatches(g, [](const Vec3& p) { return in_open_box(p, {.25, .25, .25}, {.75, .75, .75}); }), 0u);
}

TEST(VoxelizeEdgeCases, RaysThroughSharedVertices) {
  // Octahedron |x-.5| + |y-.5| + |z-.5| < 0.35 with vertices on the row y = z = 0.5: that ray
  // passes exactly through two vertices, each shared by four triangles. Other rows hit edges.
  Grid3D g(5, 5, 5, 1, 1, 1);
  const auto stats = voxelize_mesh(g, make_octahedron({0.5, 0.5, 0.5}, 0.35));
  EXPECT_GT(stats.ambiguous_crossings, 0u);
  EXPECT_EQ(stats.unresolved_cells, 0u);
  EXPECT_EQ(mismatches(g, [](const Vec3& p) {
              return std::abs(p.x - .5) + std::abs(p.y - .5) + std::abs(p.z - .5) < 0.35;
            }),
            0u);
  EXPECT_EQ(g.cell(1, 2, 2), CellType::Solid);
  EXPECT_EQ(g.cell(3, 2, 2), CellType::Solid);
  EXPECT_EQ(g.cell(0, 2, 2), CellType::Fluid);
}

TEST(VoxelizeEdgeCases, CentersExactlyOnTheSurfaceAreSolid) {
  // dx = 1/8: the box faces at 0.1875 and 0.6875 pass exactly through cell centers.
  Grid3D g(8, 8, 8, 1, 1, 1);
  const auto stats = voxelize_mesh(g, make_box({0.1875, 0.1875, 0.1875}, {0.6875, 0.6875, 0.6875}));
  EXPECT_GT(stats.surface_cells, 0u);
  EXPECT_EQ(stats.solid_cells, 125u);  // 5 centers per axis in the closed box
  EXPECT_EQ(g.cell(1, 3, 3), CellType::Solid);  // x = 0.1875, on the face
  EXPECT_EQ(g.cell(5, 5, 5), CellType::Solid);  // corner (0.6875)^3
  EXPECT_EQ(g.cell(0, 3, 3), CellType::Fluid);
  EXPECT_EQ(g.cell(6, 3, 3), CellType::Fluid);
}

TEST(VoxelizeEdgeCases, CentersVeryCloseToTheSurface) {
  Grid3D g(8, 8, 8, 1, 1, 1);  // center of cell i = 2 at x = 0.3125
  for (const double offset : {1e-7, -1e-7}) {
    const double lo = 0.3125 + offset;
    voxelize_mesh(g, make_box({lo, 0.3, 0.3}, {0.7, 0.7, 0.7}));
    EXPECT_EQ(g.cell(2, 3, 3), offset > 0 ? CellType::Fluid : CellType::Solid) << offset;
    EXPECT_EQ(g.cell(3, 3, 3), CellType::Solid);
  }
}

TEST(VoxelizeEdgeCases, DegenerateTrianglesDoNotChangeTheResult) {
  auto mesh = make_box({0.25, 0.25, 0.25}, {0.75, 0.75, 0.75});
  const Vec3 c = mesh.triangles[0].a;
  mesh.triangles.push_back({c, c, mesh.triangles[0].b});
  const auto report = validate_mesh(mesh);
  EXPECT_EQ(report.degenerate_removed, 1u);
  Grid3D g(8, 8, 8, 1, 1, 1);
  EXPECT_EQ(voxelize_mesh(g, mesh).solid_cells, 64u);
}

// --- sphere convergence -----------------------------------------------------------------------

TEST(VoxelizeSphere, VolumeConvergesWithResolution) {
  const double r = 0.3;
  const auto sphere = make_sphere({0.5, 0.5, 0.5}, r, 48, 96);
  const double v_mesh = signed_volume(sphere);
  const double v_exact = 4.0 / 3.0 * std::numbers::pi * r * r * r;
  std::printf("\n  sphere r=%.2f: mesh volume %.6e, analytic %.6e (faceting %.3f%%)\n", r, v_mesh, v_exact,
              100 * std::abs(v_mesh - v_exact) / v_exact);
  std::printf("  %6s %12s %14s %12s %10s\n", "N", "solid", "V_voxel", "rel.error", "ms");

  std::vector<double> errors;
  for (const std::size_t n : {16u, 32u, 64u, 128u}) {
    Grid3D g(n, n, n, 1, 1, 1);
    const auto s = voxelize_mesh(g, sphere);
    const double v_vox = static_cast<double>(s.solid_cells) * g.cell_volume();
    const double err = std::abs(v_vox - v_mesh) / v_mesh;
    errors.push_back(err);
    std::printf("  %6zu %12zu %14.6e %11.4f%% %10.2f\n", n, s.solid_cells, v_vox, 100 * err,
                s.bvh_build_ms + s.classify_ms);
    EXPECT_EQ(s.unresolved_cells, 0u);
    // Every cell clearly away from the surface (beyond the facet deviation) is correct.
    EXPECT_EQ(mismatches(g,
                         [&](const Vec3& p) -> std::optional<bool> {
                           const double d = std::hypot(p.x - .5, p.y - .5, p.z - .5);
                           if (std::abs(d - r) < 2e-3) return std::nullopt;
                           return d < r;
                         }),
              0u);
  }
  // Coarse grids are inaccurate; the error must shrink as the grid is refined.
  EXPECT_LT(errors[3], errors[0]);
  EXPECT_LT(errors[2], 0.02);
  EXPECT_LT(errors[3], 0.01);
}

// --- transformed geometry ---------------------------------------------------------------------

TEST(VoxelizeTransformed, TranslationRotationScaleAndUnits) {
  // Raw box 400 x 200 x 100 mm, pivot at its center.
  const auto raw = make_box({0, 0, 0}, {400, 200, 100});
  const Vec3 pivot{200, 100, 50}, scale{1.0, 1.5, 2.0}, rot{10, 30, -20}, pos{0.5, 0.45, 0.55};
  const auto m = compose(0.001, pivot, scale, rot, pos);
  const auto world = transformed(raw, m);
  EXPECT_NEAR(signed_volume(world), 0.4 * 0.3 * 0.2, 1e-12);  // det = 1.5 * 2 * (mm -> m)^3

  // Inverse of p_world = A p_raw + t, with A = R diag(s u): p_raw = diag(1/(s u)) R^T (p - t).
  const auto rot_only = compose(1.0, {0, 0, 0}, {1, 1, 1}, rot, {0, 0, 0});
  auto to_raw = [&](const Vec3& p) {
    const Vec3 d{p.x - m[3], p.y - m[7], p.z - m[11]};
    const double s[3] = {scale.x * 0.001, scale.y * 0.001, scale.z * 0.001};
    Vec3 q{};
    q.x = (rot_only[0] * d.x + rot_only[4] * d.y + rot_only[8] * d.z) / s[0];
    q.y = (rot_only[1] * d.x + rot_only[5] * d.y + rot_only[9] * d.z) / s[1];
    q.z = (rot_only[2] * d.x + rot_only[6] * d.y + rot_only[10] * d.z) / s[2];
    return q;
  };
  Grid3D g(50, 45, 55, 1.0, 0.9, 1.1);
  const auto stats = voxelize_mesh(g, world);
  EXPECT_GT(stats.solid_cells, 0u);
  EXPECT_EQ(mismatches(g, [&](const Vec3& p) { return in_open_box(to_raw(p), {0, 0, 0}, {400, 200, 100}); }), 0u);
}

TEST(VoxelizeTransformed, TranslationByWholeCellsShiftsTheMask) {
  Grid3D a(8, 8, 8, 1, 1, 1), b(8, 8, 8, 1, 1, 1);
  voxelize_mesh(a, make_box({0.25, 0.25, 0.25}, {0.75, 0.75, 0.75}));
  voxelize_mesh(b, make_box({0.375, 0.25, 0.5}, {0.875, 0.75, 1.0}));  // +1 cell in x, +2 in z
  for (std::size_t k = 0; k + 2 < 8; ++k)
    for (std::size_t j = 0; j < 8; ++j)
      for (std::size_t i = 0; i + 1 < 8; ++i) EXPECT_EQ(a.cell(i, j, k), b.cell(i + 1, j, k + 2));
}

TEST(VoxelizeTransformed, RepeatedVoxelizationIsIdentical) {
  const auto raw = make_sphere({0, 0, 0}, 150, 16, 32);  // mm
  const auto m = compose(0.001, {0, 0, 0}, {1.2, 0.8, 1.0}, {0, 45, 0}, {0.5, 0.5, 0.5});
  Grid3D g(32, 32, 32, 1, 1, 1);
  voxelize_mesh(g, transformed(raw, m));
  const std::vector<CellType> first(g.cells().begin(), g.cells().end());
  for (int n = 0; n < 3; ++n) {
    // Transform applied afresh from raw coordinates each time: nothing accumulates.
    voxelize_mesh(g, transformed(raw, m));
    EXPECT_TRUE(std::equal(first.begin(), first.end(), g.cells().begin()));
  }
}

// --- BVH performance and equivalence ---------------------------------------------------------

TEST(VoxelizePerformance, BvhMatchesBruteForceWithFarFewerTests) {
  const auto sphere = make_sphere({0.5, 0.5, 0.5}, 0.35, 48, 96);  // 9,024 triangles
  Grid3D bvh_grid(48, 48, 48, 1, 1, 1), brute_grid(48, 48, 48, 1, 1, 1);
  VoxelizeOptions brute;
  brute.use_bvh = false;
  const auto s_bvh = voxelize_mesh(bvh_grid, sphere);
  const auto s_brute = voxelize_mesh(brute_grid, sphere, brute);
  EXPECT_TRUE(std::equal(bvh_grid.cells().begin(), bvh_grid.cells().end(), brute_grid.cells().begin()));
  std::printf("\n  %zu triangles, 48^3: ray-triangle tests BVH %zu vs brute force %zu (%.1fx fewer); "
              "%.2f ms vs %.2f ms\n",
              sphere.triangles.size(), s_bvh.ray_triangle_tests, s_brute.ray_triangle_tests,
              static_cast<double>(s_brute.ray_triangle_tests) / static_cast<double>(s_bvh.ray_triangle_tests),
              s_bvh.bvh_build_ms + s_bvh.classify_ms, s_brute.classify_ms);
  EXPECT_LT(s_bvh.ray_triangle_tests * 50, s_brute.ray_triangle_tests);
}

TEST(VoxelizePerformance, LeafSizeDoesNotChangeTheResult) {
  const auto sphere = make_sphere({0.5, 0.5, 0.5}, 0.3, 12, 24);
  Grid3D ref(24, 24, 24, 1, 1, 1);
  voxelize_mesh(ref, sphere);
  for (std::size_t leaf : {1u, 2u, 8u, 32u}) {
    Grid3D g(24, 24, 24, 1, 1, 1);
    VoxelizeOptions o;
    o.bvh_leaf_size = leaf;
    voxelize_mesh(g, sphere, o);
    EXPECT_TRUE(std::equal(ref.cells().begin(), ref.cells().end(), g.cells().begin())) << leaf;
  }
}

TEST(VoxelizePerformance, RowsOutsideTheMeshAreSkipped) {
  Grid3D g(32, 32, 32, 1, 1, 1);
  const auto s = voxelize_mesh(g, make_box({0.1, 0.4, 0.4}, {0.9, 0.6, 0.6}));
  EXPECT_EQ(s.rows_total, 32u * 32u);
  EXPECT_GT(s.rows_skipped, s.rows_total * 9 / 10);
}

// --- engine pipeline (file -> validate -> voxelize -> metadata) --------------------------------

TEST(VoxelizePipeline, ReportsVolumeErrorAndSource) {
  const auto path = write_temp(make_box({0.25, 0.25, 0.25}, {0.75, 0.75, 0.75}), "cube.aftri");
  const auto config = mesh_config(8, 1.0, path);
  const auto built = build(config);
  ASSERT_TRUE(built.mesh.has_value());
  const auto meta = grid_metadata(built.grid, config, true, &*built.mesh);
  EXPECT_EQ(meta["geometry_source"], "mesh");
  EXPECT_EQ(meta["test_box"], nullptr);
  EXPECT_EQ(meta["classification"]["solid_count"], 64);
  EXPECT_EQ(meta["mesh"]["inside_domain"], true);
  EXPECT_EQ(meta["mesh"]["orientation"], "outward");
  EXPECT_DOUBLE_EQ(meta["mesh"]["volume"].get<double>(), 0.125);
  EXPECT_DOUBLE_EQ(meta["mesh"]["volume_relative_error"].get<double>(), 0.0);
  EXPECT_GT(meta["voxelization"]["ray_triangle_tests"].get<int>(), 0);
  EXPECT_TRUE(meta["warnings"].empty());
  std::filesystem::remove(path);
}

TEST(VoxelizePipeline, ModelPartlyOutsideTheTunnelWarns) {
  // x extends to 1.4 in a 1 m domain: in-domain centers 0.6875, 0.8125, 0.9375 are solid.
  const auto path = write_temp(make_box({0.6, 0.25, 0.25}, {1.4, 0.75, 0.75}), "outside.aftri");
  const auto config = mesh_config(8, 1.0, path);
  const auto built = build(config);
  const auto meta = grid_metadata(built.grid, config, false, &*built.mesh);
  EXPECT_EQ(meta["classification"]["solid_count"], 3 * 4 * 4);
  EXPECT_EQ(meta["mesh"]["inside_domain"], false);
  EXPECT_EQ(meta["mesh"]["volume_relative_error"], nullptr);
  ASSERT_EQ(meta["warnings"].size(), 1u);
  EXPECT_NE(meta["warnings"][0].get<std::string>().find("outside the tunnel"), std::string::npos);
  std::filesystem::remove(path);
}

TEST(VoxelizePipeline, InvalidMeshesAreErrors) {
  auto open = make_box({0.2, 0.2, 0.2}, {0.8, 0.8, 0.8});
  open.triangles.pop_back();
  const auto path = write_temp(open, "open.aftri");
  EXPECT_THROW(build(mesh_config(8, 1.0, path)), MeshError);
  write_triangle_file(path, TriangleMesh{});
  EXPECT_THROW(build(mesh_config(8, 1.0, path)), MeshError);
  std::filesystem::remove(path);
}

TEST(VoxelizePipeline, ConfigRejectsCombinedGeometrySources) {
  const auto j = nlohmann::json::parse(R"({
    "domain": {"lx": 1, "ly": 1, "lz": 1}, "resolution": {"nx": 4, "ny": 4, "nz": 4},
    "test_box": {"min": [0.1, 0.1, 0.1], "max": [0.5, 0.5, 0.5]},
    "mesh": {"triangles_file": "x.aftri"}
  })");
  try {
    (void)parse_config(j);
    FAIL() << "expected ConfigError";
  } catch (const ConfigError& e) {
    EXPECT_NE(std::string(e.what()).find("mutually exclusive"), std::string::npos);
  }
  auto ok = j;
  ok.erase("test_box");
  const auto c = parse_config(ok);
  ASSERT_TRUE(c.mesh.has_value());
  EXPECT_EQ(c.mesh->triangles_file, "x.aftri");
  EXPECT_EQ(c.mesh->bvh_leaf_size, 4u);
  ok["mesh"]["bvh_leaf_size"] = 0;
  EXPECT_THROW((void)parse_config(ok), ConfigError);
  ok["mesh"] = {{"path", "x"}};
  EXPECT_THROW((void)parse_config(ok), ConfigError);
}
