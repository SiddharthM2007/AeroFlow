#include <gtest/gtest.h>

#include <limits>
#include <set>
#include <stdexcept>

#include "aeroflow/box.hpp"
#include "aeroflow/mac_grid.hpp"
#include "aeroflow/voxelize.hpp"
#include "mesh_fixtures.hpp"

using namespace aeroflow;

// --- dimensions and indexing ------------------------------------------------------------------

TEST(MacGridLayout, FieldExtentsForNonCubicGrid) {
  const MacGrid g(3, 4, 5, 1.5, 2.0, 0.5);
  EXPECT_EQ(g.cell_count(), 60u);
  EXPECT_EQ(g.p().size(), 3u * 4u * 5u);
  EXPECT_EQ(g.u().size(), 4u * 4u * 5u);
  EXPECT_EQ(g.v().size(), 3u * 5u * 5u);
  EXPECT_EQ(g.w().size(), 3u * 4u * 6u);
  EXPECT_EQ(g.u().n0(), 4u);
  EXPECT_EQ(g.v().n1(), 5u);
  EXPECT_EQ(g.w().n2(), 6u);
  EXPECT_DOUBLE_EQ(g.dx(), 0.5);
  EXPECT_DOUBLE_EQ(g.dy(), 0.5);
  EXPECT_DOUBLE_EQ(g.dz(), 0.1);
  EXPECT_EQ(MacGrid::required_bytes(3, 4, 5), (60u + 80u + 75u + 72u) * sizeof(double));
}

TEST(MacGridLayout, EveryFieldIsXFastest) {
  const MacGrid g(3, 4, 5, 1, 1, 1);
  for (const StaggeredArray* a : {&g.p(), &g.u(), &g.v(), &g.w()}) {
    std::size_t expected = 0;
    for (std::size_t k = 0; k < a->n2(); ++k)
      for (std::size_t j = 0; j < a->n1(); ++j)
        for (std::size_t i = 0; i < a->n0(); ++i) EXPECT_EQ(a->index(i, j, k), expected++);
    EXPECT_EQ(expected, a->size());
  }
  EXPECT_EQ(g.u().index(1, 2, 3), 1u + 4u * (2u + 4u * 3u));
  EXPECT_EQ(g.v().index(1, 2, 3), 1u + 3u * (2u + 5u * 3u));
  EXPECT_EQ(g.w().index(1, 2, 3), 1u + 3u * (2u + 4u * 3u));
}

TEST(MacGridLayout, FieldsAreIndependentContiguousArrays) {
  MacGrid g(2, 2, 2, 1, 1, 1);
  g.u()(2, 1, 1) = 7.0;
  g.v()(1, 2, 1) = 8.0;
  EXPECT_EQ(g.u().data().back(), 7.0);
  EXPECT_EQ(g.v().data().back(), 8.0);
  EXPECT_EQ(g.w().data().back(), 0.0);
  EXPECT_NE(g.u().data().data(), g.v().data().data());
}

// --- positions --------------------------------------------------------------------------------

TEST(MacGridPositions, MatchTheDocumentedPlacement) {
  const MacGrid g(4, 5, 2, 2.0, 1.0, 3.0);  // dx 0.5, dy 0.2, dz 1.5
  auto expect_vec = [](const Vec3& a, const Vec3& b) {
    EXPECT_DOUBLE_EQ(a.x, b.x);
    EXPECT_DOUBLE_EQ(a.y, b.y);
    EXPECT_DOUBLE_EQ(a.z, b.z);
  };
  expect_vec(g.p_position(1, 2, 1), {0.75, 0.5, 2.25});
  expect_vec(g.u_position(1, 2, 1), {0.5, 0.5, 2.25});
  expect_vec(g.v_position(1, 2, 1), {0.75, 0.4, 2.25});
  expect_vec(g.w_position(1, 2, 1), {0.75, 0.5, 1.5});
  // Outermost faces lie on the tunnel walls.
  EXPECT_DOUBLE_EQ(g.u_position(0, 0, 0).x, 0.0);
  EXPECT_DOUBLE_EQ(g.u_position(4, 0, 0).x, 2.0);
  EXPECT_DOUBLE_EQ(g.v_position(0, 5, 0).y, 1.0);
  EXPECT_DOUBLE_EQ(g.w_position(0, 0, 2).z, 3.0);
}

TEST(MacGridPositions, PressureMatchesGrid3DCellCentersExactly) {
  const Grid3D cells(7, 3, 5, 1.7, 0.31, 2.9);
  const MacGrid mac(cells);
  EXPECT_TRUE(mac.matches(cells));
  for (std::size_t k = 0; k < 5; ++k)
    for (std::size_t j = 0; j < 3; ++j)
      for (std::size_t i = 0; i < 7; ++i) EXPECT_EQ(mac.p_position(i, j, k), cells.cell_center(i, j, k));
}

TEST(MacGridPositions, FacesSitHalfACellFromTheCenters) {
  const MacGrid g(6, 4, 3, 1.3, 0.7, 2.2);
  for (std::size_t i = 0; i < 6; ++i) {
    const Vec3 c = g.p_position(i, 2, 1);
    EXPECT_NEAR(g.u_position(i, 2, 1).x, c.x - g.dx() / 2, 1e-15);
    EXPECT_NEAR(g.u_position(i + 1, 2, 1).x, c.x + g.dx() / 2, 1e-15);
    EXPECT_EQ(g.u_position(i, 2, 1).y, c.y);
    EXPECT_EQ(g.u_position(i, 2, 1).z, c.z);
  }
}

// --- robustness -------------------------------------------------------------------------------

TEST(MacGridRobustness, SmallestValidGrid) {
  MacGrid g(1, 1, 1, 0.1, 0.2, 0.3);
  EXPECT_EQ(g.p().size(), 1u);
  EXPECT_EQ(g.u().size(), 2u);
  EXPECT_EQ(g.v().size(), 2u);
  EXPECT_EQ(g.w().size(), 2u);
  g.u().at(1, 0, 0) = 1.0;
  EXPECT_EQ(g.u()(1, 0, 0), 1.0);
}

TEST(MacGridRobustness, RejectsInvalidDimensionsAndSpacing) {
  EXPECT_THROW(MacGrid(0, 1, 1, 1, 1, 1), std::invalid_argument);
  EXPECT_THROW(MacGrid(1, 0, 1, 1, 1, 1), std::invalid_argument);
  EXPECT_THROW(MacGrid(1, 1, 0, 1, 1, 1), std::invalid_argument);
  EXPECT_THROW(MacGrid(1, 1, 1, 0.0, 1, 1), std::invalid_argument);
  EXPECT_THROW(MacGrid(1, 1, 1, 1, -2.0, 1), std::invalid_argument);
  EXPECT_THROW(MacGrid(1, 1, 1, 1, 1, std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(MacGrid(1, 1, 1, std::numeric_limits<double>::quiet_NaN(), 1, 1), std::invalid_argument);
  EXPECT_THROW(StaggeredArray(0, 1, 1), std::invalid_argument);
}

TEST(MacGridRobustness, IndexBoundsAreChecked) {
  MacGrid g(3, 4, 5, 1, 1, 1);
  EXPECT_NO_THROW((void)g.u().at(3, 3, 4));
  EXPECT_THROW((void)g.u().at(4, 0, 0), std::out_of_range);
  EXPECT_THROW((void)g.v().at(0, 5, 0), std::out_of_range);
  EXPECT_THROW((void)g.w().at(0, 0, 6), std::out_of_range);
  EXPECT_THROW((void)g.p().at(3, 0, 0), std::out_of_range);
  EXPECT_TRUE(g.v().in_bounds(2, 4, 4));
  EXPECT_FALSE(g.v().in_bounds(3, 4, 4));
}

TEST(MacGridRobustness, OverflowAndResourceLimits) {
  constexpr auto huge = std::numeric_limits<std::size_t>::max() / 2;
  EXPECT_THROW((void)MacGrid::required_bytes(huge, huge, 2), std::invalid_argument);
  EXPECT_THROW((void)MacGrid::required_bytes(std::numeric_limits<std::size_t>::max(), 1, 1), std::invalid_argument);
  EXPECT_THROW(MacGrid(huge, huge, 2, 1, 1, 1), std::invalid_argument);
  // More cells than Grid3D allows.
  EXPECT_THROW(MacGrid(1024, 1024, 1024, 1, 1, 1), std::invalid_argument);
  // 512^3 needs ~4.3 GB > 2 GiB default: rejected before allocating anything.
  EXPECT_GT(MacGrid::required_bytes(512, 512, 512), MacGrid::kDefaultMaxBytes);
  EXPECT_THROW(MacGrid(512, 512, 512, 1, 1, 1), std::invalid_argument);
  // Explicit tighter limit.
  EXPECT_THROW(MacGrid(64, 64, 64, 1, 1, 1, std::size_t{1} << 20), std::invalid_argument);
  EXPECT_NO_THROW(MacGrid(8, 8, 8, 1, 1, 1, MacGrid::required_bytes(8, 8, 8)));
}

// --- initialization ---------------------------------------------------------------------------

TEST(MacGridInit, ZeroAndUniformVelocity) {
  MacGrid g(3, 2, 2, 1, 1, 1);
  g.set_uniform_velocity({1.5, -2.0, 0.25}, FaceSelection::All);
  for (double x : g.u().data()) EXPECT_EQ(x, 1.5);
  for (double x : g.v().data()) EXPECT_EQ(x, -2.0);
  for (double x : g.w().data()) EXPECT_EQ(x, 0.25);
  g.p().fill(3.0);
  g.set_zero();
  for (const StaggeredArray* a : {&g.p(), &g.u(), &g.v(), &g.w()})
    for (double x : a->data()) EXPECT_EQ(x, 0.0);
}

TEST(MacGridInit, ActiveOnlyLeavesBlockedFacesAtZero) {
  Grid3D mask(3, 1, 1, 3, 1, 1);
  mask.set_cell(1, 0, 0, CellType::Solid);
  const auto faces = classify_faces(mask);
  MacGrid g(mask);
  g.set_uniform_velocity({2.0, 3.0, 4.0}, FaceSelection::All);  // dirty first
  g.set_uniform_velocity({1.0, 1.0, 1.0}, FaceSelection::ActiveOnly, &faces);
  EXPECT_EQ(g.u()(0, 0, 0), 1.0);  // tunnel wall, fluid
  EXPECT_EQ(g.u()(1, 0, 0), 0.0);  // obstacle wall
  EXPECT_EQ(g.u()(2, 0, 0), 0.0);  // obstacle wall
  EXPECT_EQ(g.u()(3, 0, 0), 1.0);
  EXPECT_EQ(g.v()(1, 0, 0), 0.0);  // wall face of the solid cell: SolidInterior
  EXPECT_EQ(g.v()(0, 1, 0), 1.0);
}

TEST(MacGridInit, ActiveOnlyRequiresMatchingFaceTypes) {
  MacGrid g(3, 3, 3, 1, 1, 1);
  EXPECT_THROW(g.set_uniform_velocity({1, 0, 0}, FaceSelection::ActiveOnly), std::invalid_argument);
  const auto other = classify_faces(Grid3D(4, 3, 3, 1, 1, 1));
  EXPECT_THROW(g.set_uniform_velocity({1, 0, 0}, FaceSelection::ActiveOnly, &other), std::invalid_argument);
}

// --- face classification ----------------------------------------------------------------------

TEST(FaceClassification, FullyFluidDomain) {
  const auto f = classify_faces(Grid3D(3, 4, 5, 1, 1, 1));
  EXPECT_EQ(f.u.size(), 4u * 4u * 5u);
  // Wall faces: two per row in each direction.
  const std::size_t walls = 2 * (4 * 5) + 2 * (3 * 5) + 2 * (3 * 4);
  EXPECT_EQ(f.count(FaceType::DomainBoundary), walls);
  EXPECT_EQ(f.count(FaceType::Fluid), f.u.size() + f.v.size() + f.w.size() - walls);
  EXPECT_EQ(f.count(FaceType::SolidBoundary), 0u);
  EXPECT_EQ(f.count(FaceType::SolidInterior), 0u);
  EXPECT_EQ(f.u_at(0, 2, 3), FaceType::DomainBoundary);
  EXPECT_EQ(f.u_at(3, 2, 3), FaceType::DomainBoundary);
  EXPECT_EQ(f.u_at(1, 2, 3), FaceType::Fluid);
}

TEST(FaceClassification, FullySolidDomain) {
  Grid3D mask(3, 4, 5, 1, 1, 1);
  mask.fill(CellType::Solid);
  const auto f = classify_faces(mask);
  EXPECT_EQ(f.count(FaceType::SolidInterior), f.u.size() + f.v.size() + f.w.size());
}

TEST(FaceClassification, ObstacleFacesAreNotTunnelFaces) {
  // 4 x 1 x 1 row: cells 1 and 2 solid.  wall|F|S|S|F|wall
  Grid3D mask(4, 1, 1, 4, 1, 1);
  mask.set_cell(1, 0, 0, CellType::Solid);
  mask.set_cell(2, 0, 0, CellType::Solid);
  const auto f = classify_faces(mask);
  EXPECT_EQ(f.u_at(0, 0, 0), FaceType::DomainBoundary);
  EXPECT_EQ(f.u_at(1, 0, 0), FaceType::SolidBoundary);
  EXPECT_EQ(f.u_at(2, 0, 0), FaceType::SolidInterior);  // shared by two solid cells
  EXPECT_EQ(f.u_at(3, 0, 0), FaceType::SolidBoundary);
  EXPECT_EQ(f.u_at(4, 0, 0), FaceType::DomainBoundary);
  // y/z wall faces: fluid cells -> DomainBoundary, solid cells -> SolidInterior.
  EXPECT_EQ(f.v_at(0, 0, 0), FaceType::DomainBoundary);
  EXPECT_EQ(f.v_at(1, 1, 0), FaceType::SolidInterior);
  EXPECT_EQ(f.w_at(2, 0, 1), FaceType::SolidInterior);
  EXPECT_EQ(f.w_at(3, 0, 0), FaceType::DomainBoundary);
  EXPECT_THROW((void)f.u_at(5, 0, 0), std::out_of_range);
}

TEST(FaceClassification, SolidTouchingTheTunnelWall) {
  Grid3D mask(3, 1, 1, 3, 1, 1);
  mask.set_cell(0, 0, 0, CellType::Solid);
  const auto f = classify_faces(mask);
  EXPECT_EQ(f.u_at(0, 0, 0), FaceType::SolidInterior);  // tunnel wall covered by the obstacle
  EXPECT_EQ(f.u_at(1, 0, 0), FaceType::SolidBoundary);
  EXPECT_EQ(f.u_at(3, 0, 0), FaceType::DomainBoundary);
}

TEST(FaceClassification, MatchesCellAdjacencyForAVoxelizedMesh) {
  // Brute-force check against every pair of neighbouring cells of a real voxelized sphere.
  Grid3D mask(20, 16, 18, 2.0, 1.6, 1.8);
  voxelize_mesh(mask, fixtures::make_sphere({1.0, 0.8, 0.9}, 0.55, 16, 32));
  ASSERT_GT(mask.count(CellType::Solid), 0u);
  const auto f = classify_faces(mask);
  auto solid = [&](long i, long j, long k) -> int {  // -1 outside the domain
    if (i < 0 || j < 0 || k < 0 || i >= 20 || j >= 16 || k >= 18) return -1;
    return mask.cell(static_cast<std::size_t>(i), static_cast<std::size_t>(j), static_cast<std::size_t>(k)) ==
           CellType::Solid;
  };
  auto expected = [](int a, int b) {
    if (a < 0 || b < 0) return (a < 0 ? b : a) == 1 ? FaceType::SolidInterior : FaceType::DomainBoundary;
    if (a + b == 0) return FaceType::Fluid;
    return a + b == 1 ? FaceType::SolidBoundary : FaceType::SolidInterior;
  };
  for (long k = 0; k < 18; ++k)
    for (long j = 0; j < 16; ++j)
      for (long i = 0; i <= 20; ++i) ASSERT_EQ(f.u_at(i, j, k), expected(solid(i - 1, j, k), solid(i, j, k)));
  for (long k = 0; k < 18; ++k)
    for (long j = 0; j <= 16; ++j)
      for (long i = 0; i < 20; ++i) ASSERT_EQ(f.v_at(i, j, k), expected(solid(i, j - 1, k), solid(i, j, k)));
  for (long k = 0; k <= 18; ++k)
    for (long j = 0; j < 16; ++j)
      for (long i = 0; i < 20; ++i) ASSERT_EQ(f.w_at(i, j, k), expected(solid(i, j, k - 1), solid(i, j, k)));
  EXPECT_GT(f.count(FaceType::SolidBoundary), 0u);
  EXPECT_GT(f.count(FaceType::SolidInterior), 0u);
}

TEST(FaceClassification, ProceduralBoxMask) {
  Grid3D mask(8, 8, 8, 1, 1, 1);
  mark_solid_box(mask, {{0.25, 0.25, 0.25}, {0.75, 0.75, 0.75}});  // 4^3 solid block
  const auto f = classify_faces(mask);
  EXPECT_EQ(f.count(FaceType::SolidBoundary), 6u * 4u * 4u);  // the block's surface
  EXPECT_EQ(f.count(FaceType::SolidInterior), 3u * 3u * 4u * 4u);  // 3 internal planes per axis
}
