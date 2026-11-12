#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "aeroflow/boundary.hpp"
#include "aeroflow/box.hpp"
#include "aeroflow/divergence.hpp"
#include "aeroflow/voxelize.hpp"
#include "mesh_fixtures.hpp"

using namespace aeroflow;
using BC = VelocityFaceBC;
using C = VelocityComponent;

namespace {

// Fills every face with distinct pseudo-random values so untouched faces are detectable.
void scramble(MacGrid& mac, unsigned seed = 7) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-3.0, 3.0);
  for (StaggeredArray* a : {&mac.u(), &mac.v(), &mac.w()})
    for (double& x : a->data()) x = dist(rng);
}

std::vector<double> snapshot(const MacGrid& mac) {
  std::vector<double> out;
  for (const StaggeredArray* a : {&mac.u(), &mac.v(), &mac.w()})
    out.insert(out.end(), a->data().begin(), a->data().end());
  return out;
}

// Obstacle that touches the floor (y = 0) and spans part of the inlet and outlet planes.
Grid3D obstructed_grid() {
  Grid3D mask(10, 6, 8, 2.0, 0.9, 1.6);  // dx 0.2, dy 0.15, dz 0.2 (non-cubic)
  mark_solid_box(mask, {{0.6, 0.0, 0.4}, {1.2, 0.45, 1.0}});  // i 3..5, j 0..2, k 2..4, on the floor
  mark_solid_box(mask, {{0.0, 0.6, 1.2}, {0.2, 0.9, 1.6}});   // touches the inlet plane and two walls
  mark_solid_box(mask, {{1.8, 0.0, 0.0}, {2.0, 0.3, 0.4}});   // touches the outlet plane
  return mask;
}

const BoundaryConditions kBC{5.0};

}  // namespace

// --- classification ---------------------------------------------------------------------------

TEST(BoundaryClassification, FullyFluidNonCubicDomain) {
  const Grid3D mask(7, 4, 5, 1.4, 0.6, 1.0);
  const auto map = classify_boundaries(mask);
  EXPECT_EQ(map.count(BC::Inlet), 4u * 5u);
  EXPECT_EQ(map.count(BC::Outlet), 4u * 5u);
  EXPECT_EQ(map.count(BC::SlipWall), 2u * 7u * 5u + 2u * 7u * 4u);
  EXPECT_EQ(map.count(BC::SolidWall), 0u);
  EXPECT_EQ(map.count(BC::Inactive), 0u);
  EXPECT_EQ(map.count(BC::Interior), map.u.size() + map.v.size() + map.w.size() - 40u - 126u);
  // Inlet/outlet only ever on u faces; slip walls only on v/w faces.
  for (auto b : map.v) EXPECT_TRUE(b == BC::Interior || b == BC::SlipWall);
  EXPECT_EQ(map.u_at(0, 2, 3), BC::Inlet);
  EXPECT_EQ(map.u_at(7, 2, 3), BC::Outlet);
  EXPECT_EQ(map.u_at(3, 2, 3), BC::Interior);
  EXPECT_EQ(map.v_at(3, 0, 3), BC::SlipWall);
  EXPECT_EQ(map.w_at(3, 2, 5), BC::SlipWall);
  EXPECT_THROW((void)map.u_at(8, 0, 0), std::out_of_range);
}

TEST(BoundaryClassification, ObstaclesTakePrecedenceOverTunnelWalls) {
  const auto mask = obstructed_grid();
  const auto map = classify_boundaries(mask);
  // Obstacle on the floor: its floor faces are covered (Inactive), not slip walls.
  EXPECT_EQ(map.v_at(4, 0, 3), BC::Inactive);
  EXPECT_EQ(map.v_at(4, 3, 3), BC::SolidWall);  // top of the obstacle
  EXPECT_EQ(map.u_at(3, 1, 3), BC::SolidWall);  // upstream face
  EXPECT_EQ(map.u_at(6, 1, 3), BC::SolidWall);  // downstream face
  EXPECT_EQ(map.u_at(4, 1, 3), BC::Inactive);   // inside
  // Solid on the inlet / outlet planes: those faces are not inlet or outlet.
  EXPECT_EQ(map.u_at(0, 5, 7), BC::Inactive);
  EXPECT_EQ(map.u_at(0, 0, 0), BC::Inlet);
  EXPECT_EQ(map.u_at(10, 0, 0), BC::Inactive);
  EXPECT_EQ(map.u_at(10, 5, 7), BC::Outlet);
}

// --- velocity application ---------------------------------------------------------------------

TEST(BoundaryApply, UniformInletZeroWallsAndNoPenetration) {
  const auto mask = obstructed_grid();
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  scramble(mac);
  apply_velocity_bcs(mac, map, kBC);
  for (std::size_t n = 0; n < map.u.size(); ++n) {
    if (map.u[n] == BC::Inlet) EXPECT_EQ(mac.u().data()[n], 5.0);
    if (map.u[n] == BC::SolidWall || map.u[n] == BC::Inactive) EXPECT_EQ(mac.u().data()[n], 0.0);
  }
  for (std::size_t n = 0; n < map.v.size(); ++n)
    if (is_prescribed(map.v[n])) EXPECT_EQ(mac.v().data()[n], 0.0);  // lateral walls and obstacles
  for (std::size_t n = 0; n < map.w.size(); ++n)
    if (is_prescribed(map.w[n])) EXPECT_EQ(mac.w().data()[n], 0.0);
  EXPECT_EQ(boundary_fluxes(mac, map).wall_leak, 0.0);
}

TEST(BoundaryApply, InteriorFacesAreNeverModified) {
  const auto mask = obstructed_grid();
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  scramble(mac);
  const auto before = snapshot(mac);
  apply_velocity_bcs(mac, map, kBC);
  const auto after = snapshot(mac);
  std::vector<BC> all(map.u);
  all.insert(all.end(), map.v.begin(), map.v.end());
  all.insert(all.end(), map.w.begin(), map.w.end());
  std::size_t interior = 0;
  for (std::size_t n = 0; n < all.size(); ++n) {
    if (all[n] == BC::Interior) {
      EXPECT_EQ(after[n], before[n]);
      ++interior;
    }
  }
  EXPECT_GT(interior, 0u);
}

TEST(BoundaryApply, OutletIsZeroGradientNotPrescribed) {
  const auto mask = obstructed_grid();
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  scramble(mac);
  apply_velocity_bcs(mac, map, kBC);
  for (std::size_t k = 0; k < 8; ++k)
    for (std::size_t j = 0; j < 6; ++j) {
      if (map.u_at(10, j, k) == BC::Outlet) {
        EXPECT_EQ(mac.u()(10, j, k), mac.u()(9, j, k));  // copies the last cell's upstream face
      } else {
        EXPECT_EQ(mac.u()(10, j, k), 0.0);  // blocked outlet face
      }
    }
  EXPECT_EQ(pressure_condition(BC::Outlet), PressureFaceBC::Dirichlet);
}

TEST(BoundaryApply, OneCellLongTunnelCopiesTheInlet) {
  const Grid3D mask(1, 3, 2, 0.1, 0.3, 0.2);
  MacGrid mac(mask);
  apply_velocity_bcs(mac, classify_boundaries(mask), kBC);
  for (std::size_t k = 0; k < 2; ++k)
    for (std::size_t j = 0; j < 3; ++j) EXPECT_EQ(mac.u()(1, j, k), 5.0);
}

TEST(BoundaryApply, RepeatedApplicationIsIdempotent) {
  Grid3D mask(24, 16, 16, 2.4, 1.6, 1.6);
  voxelize_mesh(mask, fixtures::make_sphere({1.0, 0.8, 0.8}, 0.4, 16, 32));
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  scramble(mac);
  apply_velocity_bcs(mac, map, kBC);
  const auto once = snapshot(mac);
  apply_velocity_bcs(mac, map, kBC);
  apply_velocity_bcs(mac, map, kBC);
  EXPECT_EQ(snapshot(mac), once);
}

TEST(BoundaryApply, CornersAndEdgesHaveOneConsistentValue) {
  // Every face has exactly one role, so edge/corner faces get one value each: the u face at
  // the inlet/floor/side corner is the inlet, the v and w faces there are slip walls.
  const Grid3D mask(5, 4, 3, 1.0, 0.8, 0.6);
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  scramble(mac);
  apply_velocity_bcs(mac, map, kBC);
  for (const auto [j, k] : {std::pair<std::size_t, std::size_t>{0, 0}, {3, 0}, {0, 2}, {3, 2}}) {
    EXPECT_EQ(map.u_at(0, j, k), BC::Inlet);
    EXPECT_EQ(mac.u()(0, j, k), 5.0);
    EXPECT_EQ(map.u_at(5, j, k), BC::Outlet);
  }
  for (const std::size_t i : {0u, 4u}) {
    EXPECT_EQ(map.v_at(i, 0, 0), BC::SlipWall);
    EXPECT_EQ(mac.v()(i, 0, 0), 0.0);
    EXPECT_EQ(map.w_at(i, 0, 0), BC::SlipWall);
    EXPECT_EQ(mac.w()(i, 0, 0), 0.0);
  }
}

TEST(BoundaryApply, FullyFluidUniformFlowConservesMassAndIsDivergenceFree) {
  const Grid3D mask(9, 5, 4, 1.8, 0.75, 0.8);
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  mac.set_uniform_velocity({5.0, 0.0, 0.0}, FaceSelection::All);
  apply_velocity_bcs(mac, map, kBC);
  const auto f = boundary_fluxes(mac, map);
  EXPECT_DOUBLE_EQ(f.inlet, 5.0 * 0.75 * 0.8);
  EXPECT_DOUBLE_EQ(f.outlet, f.inlet);
  EXPECT_EQ(f.wall_leak, 0.0);
  EXPECT_EQ(divergence_stats(compute_divergence(mac, mask), mask)->max_abs, 0.0);
}

TEST(BoundaryApply, PartiallyObstructedInflowAreaShrinks) {
  const auto mask = obstructed_grid();
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  apply_velocity_bcs(mac, map, kBC);
  const auto f = boundary_fluxes(mac, map);
  const double open = 0.9 * 1.6 - (0.3 * 0.4);  // inlet minus the blocked corner (2 x 2 cells)
  EXPECT_NEAR(f.inlet_area, open, 1e-12);
  EXPECT_NEAR(f.inlet, 5.0 * open, 1e-12);
  EXPECT_NEAR(f.outlet_area, 0.9 * 1.6 - 0.3 * 0.4, 1e-12);
}

TEST(BoundaryApply, ImportedMeshObstacleHasNoPenetration) {
  Grid3D mask(30, 20, 20, 3.0, 2.0, 2.0);
  voxelize_mesh(mask, fixtures::make_sphere({1.2, 1.0, 1.0}, 0.5, 16, 32));
  const auto map = classify_boundaries(mask);
  ASSERT_GT(map.count(BC::SolidWall), 0u);
  MacGrid mac(mask);
  scramble(mac, 11);
  apply_velocity_bcs(mac, map, kBC);
  EXPECT_EQ(boundary_fluxes(mac, map).wall_leak, 0.0);
}

// --- tangential ghost rules -------------------------------------------------------------------

TEST(BoundaryGhosts, FreeSlipWallsReflectEvenly) {
  const Grid3D mask(4, 3, 3, 1, 1, 1);
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  mac.u()(2, 0, 1) = 3.5;
  mac.u()(2, 2, 1) = -1.25;
  mac.w()(1, 0, 2) = 0.75;
  // Zero normal derivative: ghost equals the sample.
  EXPECT_EQ(tangential_neighbor(mac, map, C::U, 2, 0, 1, 1, -1), 3.5);
  EXPECT_EQ(tangential_neighbor(mac, map, C::U, 2, 2, 1, 1, +1), -1.25);
  EXPECT_EQ(tangential_neighbor(mac, map, C::W, 1, 0, 2, 1, -1), 0.75);
  // Inside the fluid the stored neighbor is returned.
  mac.u()(2, 1, 1) = 9.0;
  EXPECT_EQ(tangential_neighbor(mac, map, C::U, 2, 0, 1, 1, +1), 9.0);
}

TEST(BoundaryGhosts, InletHasZeroTransverseVelocityOutletZeroGradient) {
  const Grid3D mask(4, 3, 3, 1, 1, 1);
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  mac.v()(0, 1, 1) = 2.0;
  mac.w()(3, 1, 1) = -0.5;
  const double inlet_ghost = tangential_neighbor(mac, map, C::V, 0, 1, 1, 0, -1);
  EXPECT_EQ(inlet_ghost, -2.0);
  EXPECT_EQ((inlet_ghost + mac.v()(0, 1, 1)) / 2, 0.0);  // v = 0 on the inlet plane
  EXPECT_EQ(tangential_neighbor(mac, map, C::W, 3, 1, 1, 0, +1), -0.5);  // outlet: zero gradient
}

TEST(BoundaryGhosts, NoSlipAtObstaclesPlacesZeroOnTheVoxelFace) {
  // 3 x 3 x 1 grid with the middle-top cell row solid: u(1,1,0) has an Inactive neighbor above.
  Grid3D mask(3, 3, 1, 3, 3, 1);
  mask.set_cell(0, 2, 0, CellType::Solid);
  mask.set_cell(1, 2, 0, CellType::Solid);
  mask.set_cell(2, 2, 0, CellType::Solid);
  const auto map = classify_boundaries(mask);
  ASSERT_EQ(map.u_at(1, 2, 0), BC::Inactive);
  MacGrid mac(mask);
  apply_velocity_bcs(mac, map, kBC);
  mac.u()(1, 1, 0) = 4.0;
  const double ghost = tangential_neighbor(mac, map, C::U, 1, 1, 0, 1, +1);
  EXPECT_EQ(ghost, -4.0);
  EXPECT_EQ((ghost + mac.u()(1, 1, 0)) / 2, 0.0);  // u = 0 at y = 2 (the obstacle's face)
  // A SolidWall neighbor (on an obstacle face) is returned as stored (0 after the BCs).
  Grid3D step(3, 2, 1, 3, 2, 1);
  step.set_cell(1, 1, 0, CellType::Solid);
  const auto step_map = classify_boundaries(step);
  ASSERT_EQ(step_map.u_at(1, 1, 0), BC::SolidWall);
  MacGrid step_mac(step);
  apply_velocity_bcs(step_mac, step_map, kBC);
  step_mac.u()(1, 0, 0) = 4.0;
  EXPECT_EQ(tangential_neighbor(step_mac, step_map, C::U, 1, 0, 0, 1, +1), 0.0);
}

TEST(BoundaryGhosts, RejectsNormalAxesAndSolidSamples) {
  Grid3D mask(3, 3, 3, 1, 1, 1);
  mask.set_cell(1, 1, 1, CellType::Solid);
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  EXPECT_THROW((void)tangential_neighbor(mac, map, C::U, 1, 1, 1, 0, 1), std::invalid_argument);
  EXPECT_THROW((void)tangential_neighbor(mac, map, C::V, 1, 1, 1, 1, 1), std::invalid_argument);
  EXPECT_THROW((void)tangential_neighbor(mac, map, C::U, 1, 1, 0, 1, 2), std::invalid_argument);
  // v(1,1,1) lies between fluid cell (1,0,1) and solid cell (1,1,1): a SolidWall sample is allowed.
  EXPECT_NO_THROW((void)tangential_neighbor(mac, map, C::V, 1, 1, 1, 0, 1));
  Grid3D two(3, 3, 3, 1, 1, 1);
  two.set_cell(1, 1, 1, CellType::Solid);
  two.set_cell(1, 2, 1, CellType::Solid);
  const auto two_map = classify_boundaries(two);
  ASSERT_EQ(two_map.v_at(1, 2, 1), BC::Inactive);
  EXPECT_THROW((void)tangential_neighbor(MacGrid(two), two_map, C::V, 1, 2, 1, 0, 1), std::invalid_argument);
}

// --- pressure semantics -----------------------------------------------------------------------

TEST(BoundaryPressure, FaceConditionsAndOutletGauge) {
  EXPECT_EQ(pressure_condition(BC::Interior), PressureFaceBC::Coupled);
  EXPECT_EQ(pressure_condition(BC::Outlet), PressureFaceBC::Dirichlet);
  for (auto b : {BC::Inlet, BC::SlipWall, BC::SolidWall, BC::Inactive})
    EXPECT_EQ(pressure_condition(b), PressureFaceBC::Neumann);
  const double p = 12.5;
  EXPECT_EQ((outlet_ghost_pressure(p) + p) / 2, 0.0);  // p = 0 exactly on the outlet plane
  EXPECT_TRUE(is_prescribed(BC::Inlet));
  EXPECT_FALSE(is_prescribed(BC::Outlet));
  EXPECT_FALSE(is_prescribed(BC::Interior));
}

// --- invalid configurations -------------------------------------------------------------------

TEST(BoundaryValidation, RejectsInvalidConfigurations) {
  const Grid3D fluid(4, 3, 3, 1, 1, 1);
  const auto map = classify_boundaries(fluid);
  EXPECT_THROW(validate_boundaries(map, {std::numeric_limits<double>::quiet_NaN()}), BoundaryError);
  EXPECT_THROW(validate_boundaries(map, {std::numeric_limits<double>::infinity()}), BoundaryError);
  EXPECT_THROW(validate_boundaries(map, {-1.0}), BoundaryError);
  EXPECT_NO_THROW(validate_boundaries(map, {0.0}));

  Grid3D solid(4, 3, 3, 1, 1, 1);
  solid.fill(CellType::Solid);
  EXPECT_THROW(validate_boundaries(classify_boundaries(solid), kBC), BoundaryError);

  Grid3D blocked(4, 3, 3, 1, 1, 1);
  mark_solid_box(blocked, {{0.75, 0.0, 0.0}, {1.0, 1.0, 1.0}});  // last column solid: no outlet
  try {
    validate_boundaries(classify_boundaries(blocked), kBC);
    FAIL() << "expected BoundaryError";
  } catch (const BoundaryError& e) {
    EXPECT_NE(std::string(e.what()).find("no open outlet"), std::string::npos);
  }

  MacGrid other(4, 3, 4, 1, 1, 1);
  EXPECT_THROW(apply_velocity_bcs(other, map, kBC), BoundaryError);
  EXPECT_THROW((void)boundary_fluxes(other, map), BoundaryError);
  auto broken = map;
  broken.v.pop_back();
  EXPECT_THROW(validate_boundaries(broken, kBC), BoundaryError);
}
