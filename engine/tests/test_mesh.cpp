#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <set>

#include "aeroflow/bvh.hpp"
#include "aeroflow/mesh.hpp"
#include "aeroflow/ray.hpp"
#include "mesh_fixtures.hpp"

using namespace aeroflow;
using fixtures::make_box;
using fixtures::make_sphere;

namespace {

std::string temp_path(const char* name) {
  return (std::filesystem::temp_directory_path() / (std::string("aeroflow_") + name)).string();
}

template <typename F>
void expect_mesh_error(F&& f, const std::string& fragment) {
  try {
    f();
    FAIL() << "expected MeshError containing '" << fragment << "'";
  } catch (const MeshError& e) {
    EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos) << e.what();
  }
}

}  // namespace

// --- file format ------------------------------------------------------------------------------

TEST(TriangleFile, RoundTrips) {
  const auto mesh = make_box({0, 0, 0}, {1, 2, 3});
  const auto path = temp_path("roundtrip.aftri");
  write_triangle_file(path, mesh);
  EXPECT_EQ(std::filesystem::file_size(path), 16u + 12u * 72u);
  const auto back = read_triangle_file(path);
  ASSERT_EQ(back.triangles.size(), 12u);
  for (std::size_t n = 0; n < 12; ++n) {
    EXPECT_EQ(back.triangles[n].a, mesh.triangles[n].a);
    EXPECT_EQ(back.triangles[n].c, mesh.triangles[n].c);
  }
  std::filesystem::remove(path);
}

TEST(TriangleFile, RejectsCorruptFiles) {
  const auto path = temp_path("corrupt.aftri");
  { std::ofstream(path, std::ios::binary) << "NOTMAGIC12345678"; }
  expect_mesh_error([&] { read_triangle_file(path); }, "invalid header");

  write_triangle_file(path, make_box({0, 0, 0}, {1, 1, 1}));
  std::filesystem::resize_file(path, std::filesystem::file_size(path) - 8);
  expect_mesh_error([&] { read_triangle_file(path); }, "does not match its triangle count");

  expect_mesh_error([&] { read_triangle_file(temp_path("missing.aftri")); }, "cannot open");
  std::filesystem::remove(path);
}

// --- validation -------------------------------------------------------------------------------

TEST(MeshValidation, AcceptsClosedMeshes) {
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  const auto r = validate_mesh(box);
  EXPECT_EQ(r.triangles, 12u);
  EXPECT_EQ(r.unique_vertices, 8u);
  EXPECT_TRUE(r.consistently_oriented);
  EXPECT_TRUE(r.warnings.empty());

  auto sphere = make_sphere({0, 0, 0}, 1, 12, 24);
  EXPECT_NO_THROW(validate_mesh(sphere));
}

TEST(MeshValidation, RejectsEmpty) {
  TriangleMesh empty;
  expect_mesh_error([&] { validate_mesh(empty); }, "empty");
}

TEST(MeshValidation, RejectsNonFinite) {
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  box.triangles[3].b.y = std::numeric_limits<double>::quiet_NaN();
  expect_mesh_error([&] { validate_mesh(box); }, "non-finite");
  box.triangles[3].b.y = std::numeric_limits<double>::infinity();
  expect_mesh_error([&] { validate_mesh(box); }, "non-finite");
}

TEST(MeshValidation, RejectsOpenMesh) {
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  box.triangles.pop_back();
  expect_mesh_error([&] { validate_mesh(box); }, "not watertight: 3 open boundary edges");
}

TEST(MeshValidation, RejectsNearlyClosedMeshWithoutWelding) {
  // A 1e-9 m gap is not welded: exact matching only.
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  box.triangles[0].a.x += 1e-9;
  expect_mesh_error([&] { validate_mesh(box); }, "not watertight");
}

TEST(MeshValidation, RejectsNonManifoldEdges) {
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  // A fin attached to an existing edge -> that edge is used three times.
  const Triangle& t = box.triangles[0];
  box.triangles.push_back({t.a, t.b, {-1, -1, -1}});
  expect_mesh_error([&] { validate_mesh(box); }, "non-manifold");
}

TEST(MeshValidation, RejectsDuplicateTriangles) {
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  box.triangles.push_back(box.triangles[5]);
  expect_mesh_error([&] { validate_mesh(box); }, "1 duplicate triangle");
  // Same triangle with reversed winding is still a duplicate face.
  box = make_box({0, 0, 0}, {1, 1, 1});
  box.triangles.push_back({box.triangles[5].a, box.triangles[5].c, box.triangles[5].b});
  expect_mesh_error([&] { validate_mesh(box); }, "duplicate");
}

TEST(MeshValidation, RemovesDegenerateTrianglesWithWarning) {
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  const Vec3 corner = box.triangles[0].a;
  box.triangles.push_back({corner, corner, box.triangles[0].b});  // repeated vertex
  box.triangles.push_back({{0, 0, 0}, {0.5, 0, 0}, {1, 0, 0}});    // collinear on an edge
  const auto r = validate_mesh(box);
  EXPECT_EQ(r.input_triangles, 14u);
  EXPECT_EQ(r.degenerate_removed, 2u);
  EXPECT_EQ(box.triangles.size(), 12u);
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("2 degenerate"), std::string::npos);
}

TEST(MeshValidation, WarnsOnInconsistentWinding) {
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  std::swap(box.triangles[2].b, box.triangles[2].c);
  const auto r = validate_mesh(box);
  EXPECT_FALSE(r.consistently_oriented);
  EXPECT_EQ(r.inconsistent_edges, 3u);
  ASSERT_EQ(r.warnings.size(), 1u);
}

// --- volume -----------------------------------------------------------------------------------

TEST(MeshVolume, BoxIsExact) {
  EXPECT_NEAR(signed_volume(make_box({1, 2, 3}, {3, 3, 3.5})), 2.0 * 1.0 * 0.5, 1e-14);
  // Far from the origin: the reference-point formulation avoids cancellation.
  EXPECT_NEAR(signed_volume(make_box({1e4, 1e4, 1e4}, {1e4 + 0.1, 1e4 + 0.2, 1e4 + 0.3})), 0.006, 1e-12);
}

TEST(MeshVolume, InvertedWindingIsNegative) {
  auto box = make_box({0, 0, 0}, {1, 1, 1});
  for (auto& t : box.triangles) std::swap(t.b, t.c);
  EXPECT_NEAR(signed_volume(box), -1.0, 1e-14);
}

TEST(MeshVolume, SphereApproachesAnalytic) {
  const double exact = 4.0 / 3.0 * std::numbers::pi;
  const double coarse = signed_volume(make_sphere({0, 0, 0}, 1, 8, 16));
  const double fine = signed_volume(make_sphere({0, 0, 0}, 1, 64, 128));
  EXPECT_LT(coarse, exact);  // inscribed polyhedron
  EXPECT_LT(std::abs(fine - exact) / exact, 2e-3);
  EXPECT_LT(std::abs(fine - exact), std::abs(coarse - exact));
}

// --- ray / triangle ---------------------------------------------------------------------------

TEST(RayTriangle, ClassifiesHitsMissesAndBoundaries) {
  const Triangle tri{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // in the x = 0 plane
  const Vec3 dir{1, 0, 0};
  const RayTolerances tol;

  auto hit = intersect_ray_triangle({-2, 0.25, 0.25}, dir, tri, tol);
  EXPECT_EQ(hit.kind, HitKind::Hit);
  EXPECT_DOUBLE_EQ(hit.t, 2.0);

  EXPECT_EQ(intersect_ray_triangle({-2, 0.75, 0.75}, dir, tri, tol).kind, HitKind::Miss);
  EXPECT_EQ(intersect_ray_triangle({-2, -0.1, 0.5}, dir, tri, tol).kind, HitKind::Miss);

  // Edges and vertices -> Ambiguous (never silently counted or dropped).
  EXPECT_EQ(intersect_ray_triangle({-2, 0.5, 0.5}, dir, tri, tol).kind, HitKind::Ambiguous);  // hypotenuse
  EXPECT_EQ(intersect_ray_triangle({-2, 0.0, 0.3}, dir, tri, tol).kind, HitKind::Ambiguous);  // leg
  EXPECT_EQ(intersect_ray_triangle({-2, 0.0, 0.0}, dir, tri, tol).kind, HitKind::Ambiguous);  // vertex
  EXPECT_EQ(intersect_ray_triangle({-2, 0.3, 1e-12}, dir, tri, tol).kind, HitKind::Ambiguous);

  // Behind the origin still reports t (< 0); callers decide what counts.
  EXPECT_LT(intersect_ray_triangle({2, 0.25, 0.25}, dir, tri, tol).t, 0.0);
}

TEST(RayTriangle, ParallelRays) {
  const Triangle tri{{0, 0, 0}, {1, 0, 0}, {0, 0, 1}};  // in the y = 0 plane
  const RayTolerances tol;
  // Parallel, off the plane: clean miss.
  EXPECT_EQ(intersect_ray_triangle({-1, 0.5, 0.2}, {1, 0, 0}, tri, tol).kind, HitKind::Miss);
  // Lying in the plane: grazing -> Ambiguous with NaN t.
  const auto in_plane = intersect_ray_triangle({-1, 0.0, 0.2}, {1, 0, 0}, tri, tol);
  EXPECT_EQ(in_plane.kind, HitKind::Ambiguous);
  EXPECT_TRUE(std::isnan(in_plane.t));
}

TEST(RayTriangle, DegenerateTriangleNeverHits) {
  const Triangle line{{0, 0, 0}, {0, 1, 0}, {0, 2, 0}};
  const Triangle point{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  const RayTolerances tol;
  EXPECT_EQ(intersect_ray_triangle({-1, 0.5, 0}, {1, 0, 0}, line, tol).kind, HitKind::Miss);
  EXPECT_EQ(intersect_ray_triangle({-1, 0, 0}, {1, 0, 0}, point, tol).kind, HitKind::Miss);
}

// --- BVH --------------------------------------------------------------------------------------

TEST(Bvh, ReturnsEveryTriangleTheRayCanHit) {
  const auto mesh = make_sphere({0.5, 0.5, 0.5}, 0.3, 16, 32);
  const Bvh bvh(mesh, 4, 1e-12);
  EXPECT_GT(bvh.nodes().size(), 1u);
  EXPECT_LE(bvh.depth(), 20u);
  const RayTolerances tol;
  for (double y = 0.15; y < 0.86; y += 0.0371) {
    for (double z = 0.15; z < 0.86; z += 0.0413) {
      const Vec3 o{-1, y, z}, d{1, 0, 0};
      std::set<std::uint32_t> candidates;
      bvh.traverse(o, d, 0.0, [&](std::uint32_t n) { candidates.insert(n); });
      for (std::uint32_t n = 0; n < mesh.triangles.size(); ++n) {
        if (intersect_ray_triangle(o, d, mesh.triangles[n], tol).kind != HitKind::Miss) {
          EXPECT_TRUE(candidates.count(n)) << "BVH culled a hit triangle " << n;
        }
      }
    }
  }
}

TEST(Bvh, LeavesCoverAllTrianglesOnce) {
  const auto mesh = make_sphere({0, 0, 0}, 1, 10, 20);
  for (std::size_t leaf : {1u, 4u, 16u}) {
    const Bvh bvh(mesh, leaf, 0.0);
    std::size_t covered = 0;
    for (const auto& node : bvh.nodes()) {
      if (node.count > 0) {
        EXPECT_LE(node.count, std::max<std::size_t>(leaf, 1));
        covered += node.count;
      }
    }
    EXPECT_EQ(covered, mesh.triangles.size());
  }
}

TEST(Bvh, RayBoxHandlesZeroDirectionComponents) {
  const Aabb box{{0, 0, 0}, {1, 1, 1}};
  EXPECT_TRUE(ray_hits_box({-1, 0.5, 0.5}, {1, 0, 0}, 0, box));
  EXPECT_TRUE(ray_hits_box({-1, 1.0, 0.0}, {1, 0, 0}, 0, box));  // along an edge
  EXPECT_FALSE(ray_hits_box({-1, 1.5, 0.5}, {1, 0, 0}, 0, box));
  EXPECT_FALSE(ray_hits_box({2, 0.5, 0.5}, {1, 0, 0}, 0, box));  // box behind the ray
  EXPECT_TRUE(ray_hits_box({0.5, 0.5, 0.5}, {0.3, -0.2, 0.9}, -1e-9, box));
}
