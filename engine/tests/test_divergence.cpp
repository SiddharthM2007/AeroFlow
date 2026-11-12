#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <functional>
#include <stdexcept>

#include "aeroflow/box.hpp"
#include "aeroflow/divergence.hpp"
#include "aeroflow/voxelize.hpp"
#include "mesh_fixtures.hpp"

using namespace aeroflow;

namespace {

using Fn = std::function<double(const Vec3&)>;

// Samples analytical velocity components at their own face positions.
void sample(MacGrid& g, const Fn& fu, const Fn& fv, const Fn& fw) {
  for (std::size_t k = 0; k < g.u().n2(); ++k)
    for (std::size_t j = 0; j < g.u().n1(); ++j)
      for (std::size_t i = 0; i < g.u().n0(); ++i) g.u()(i, j, k) = fu(g.u_position(i, j, k));
  for (std::size_t k = 0; k < g.v().n2(); ++k)
    for (std::size_t j = 0; j < g.v().n1(); ++j)
      for (std::size_t i = 0; i < g.v().n0(); ++i) g.v()(i, j, k) = fv(g.v_position(i, j, k));
  for (std::size_t k = 0; k < g.w().n2(); ++k)
    for (std::size_t j = 0; j < g.w().n1(); ++j)
      for (std::size_t i = 0; i < g.w().n0(); ++i) g.w()(i, j, k) = fw(g.w_position(i, j, k));
}

double max_abs_fluid(const DivergenceField& d, const Grid3D& mask) {
  const auto s = divergence_stats(d, mask);
  return s ? s->max_abs : 0.0;
}

// Cells with no SolidBoundary/SolidInterior face (i.e. not touching an obstacle).
bool unobstructed(const FaceTypes& f, std::size_t i, std::size_t j, std::size_t k) {
  return is_active(f.u_at(i, j, k)) && is_active(f.u_at(i + 1, j, k)) && is_active(f.v_at(i, j, k)) &&
         is_active(f.v_at(i, j + 1, k)) && is_active(f.w_at(i, j, k)) && is_active(f.w_at(i, j, k + 1));
}

}  // namespace

// --- basic fields -----------------------------------------------------------------------------

TEST(Divergence, ZeroVelocityGivesZero) {
  const Grid3D mask(5, 4, 3, 1.5, 2.8, 3.3);
  MacGrid g(mask);
  const auto d = compute_divergence(g, mask);
  EXPECT_EQ(d.fluid_cells, 60u);
  for (double x : d.values.data()) EXPECT_EQ(x, 0.0);
  const auto s = divergence_stats(d, mask);
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->max_abs, 0.0);
  EXPECT_EQ(s->rms, 0.0);
}

TEST(Divergence, UniformVelocityIsExactlyDivergenceFree) {
  const Grid3D mask(5, 4, 3, 1.5, 2.8, 3.3);  // dx 0.3, dy 0.7, dz 1.1
  MacGrid g(mask);
  g.set_uniform_velocity({12.5, -3.25, 0.75}, FaceSelection::All);
  const auto d = compute_divergence(g, mask);
  for (double x : d.values.data()) EXPECT_EQ(x, 0.0);
}

TEST(Divergence, LinearFieldGivesItsTrace) {
  // u = a x, v = b y, w = c z with unequal, non-unit spacing: div = a + b + c in every cell.
  const Grid3D mask(5, 4, 3, 1.5, 2.8, 3.3);  // dx 0.3, dy 0.7, dz 1.1
  MacGrid g(mask);
  const double a = 2.5, b = -0.75, c = 4.0;
  sample(g, [&](const Vec3& p) { return a * p.x; }, [&](const Vec3& p) { return b * p.y; },
         [&](const Vec3& p) { return c * p.z; });
  const auto d = compute_divergence(g, mask);
  for (double x : d.values.data()) EXPECT_NEAR(x, a + b + c, 1e-12);
  const auto s = divergence_stats(d, mask);
  EXPECT_NEAR(s->mean_abs, a + b + c, 1e-12);
  EXPECT_NEAR(s->rms, a + b + c, 1e-12);
}

TEST(Divergence, RotationFieldIsDivergenceFree) {
  // u = a y, v = -a x, w = 0: analytically and discretely divergence-free.
  const Grid3D mask(6, 5, 4, 1.8, 1.25, 0.6);
  MacGrid g(mask);
  const double a = 3.7;
  sample(g, [&](const Vec3& p) { return a * p.y; }, [&](const Vec3& p) { return -a * p.x; },
         [](const Vec3&) { return 0.0; });
  EXPECT_LE(max_abs_fluid(compute_divergence(g, mask), mask), 1e-13);
}

TEST(Divergence, DiscreteCurlOfStreamFunctionIsDivergenceFreeToRoundoff) {
  // u = dpsi/dy, v = -dpsi/dx taken as MAC differences of psi at cell edges: the discrete
  // divergence then cancels identically, for any psi, up to floating-point rounding.
  const Grid3D mask(12, 9, 4, 1.2, 0.9, 0.5);
  MacGrid g(mask);
  auto psi = [](double x, double y) { return std::sin(3 * x) * std::cos(2 * y) + x * x * y; };
  const double dx = g.dx(), dy = g.dy();
  for (std::size_t k = 0; k < 4; ++k) {
    for (std::size_t j = 0; j < 9; ++j)
      for (std::size_t i = 0; i <= 12; ++i) {
        const double x = static_cast<double>(i) * dx;
        g.u()(i, j, k) = (psi(x, static_cast<double>(j + 1) * dy) - psi(x, static_cast<double>(j) * dy)) / dy;
      }
    for (std::size_t j = 0; j <= 9; ++j)
      for (std::size_t i = 0; i < 12; ++i) {
        const double y = static_cast<double>(j) * dy;
        g.v()(i, j, k) = -(psi(static_cast<double>(i + 1) * dx, y) - psi(static_cast<double>(i) * dx, y)) / dx;
      }
  }
  EXPECT_LE(max_abs_fluid(compute_divergence(g, mask), mask), 1e-11);
}

TEST(Divergence, SampledDivergenceFreeFieldConvergesAtSecondOrder) {
  // u = sin x cos y, v = -cos x sin y (div = 0) sampled pointwise with dx != dy: the discrete
  // divergence is a truncation error that must shrink as O(h^2).
  std::printf("\n  %6s %14s %8s\n", "N", "max|div|", "order");
  double prev = 0.0;
  for (const std::size_t n : {8u, 16u, 32u, 64u}) {
    const Grid3D mask(n, 2 * n, 1, 2.0, 1.5, 0.1);
    MacGrid g(mask);
    sample(g, [](const Vec3& p) { return std::sin(p.x) * std::cos(p.y); },
           [](const Vec3& p) { return -std::cos(p.x) * std::sin(p.y); }, [](const Vec3&) { return 0.0; });
    const double err = max_abs_fluid(compute_divergence(g, mask), mask);
    if (prev > 0) {
      const double order = std::log2(prev / err);
      std::printf("  %6zu %14.6e %8.3f\n", n, err, order);
      EXPECT_NEAR(order, 2.0, 0.1);
    } else {
      std::printf("  %6zu %14.6e %8s\n", n, err, "-");
    }
    prev = err;
  }
}

// --- solids -----------------------------------------------------------------------------------

TEST(DivergenceSolids, SolidCellsAreZeroAndInvalid) {
  Grid3D mask(4, 3, 3, 1, 1, 1);
  mark_solid_box(mask, {{0.25, 0.0, 0.0}, {0.75, 1.0, 1.0}});  // i = 1, 2 solid
  MacGrid g(mask);
  sample(g, [](const Vec3& p) { return 5 * p.x * p.x; }, [](const Vec3&) { return 0.0; },
         [](const Vec3&) { return 0.0; });
  const auto d = compute_divergence(g, mask);
  EXPECT_EQ(d.fluid_cells, 2u * 3u * 3u);
  EXPECT_EQ(d.fluid_cells, count_fluid_cells(mask));
  for (std::size_t k = 0; k < 3; ++k)
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_EQ(d.values(1, j, k), 0.0);
      EXPECT_EQ(d.values(2, j, k), 0.0);
      EXPECT_NE(d.values(0, j, k), 0.0);
    }
}

TEST(DivergenceSolids, UniformFlowOnActiveFacesOnlyShowsObstacleCells) {
  // ActiveOnly zeroes blocked faces, so fluid cells next to an obstacle see a jump in normal
  // velocity (nonzero divergence); unobstructed cells stay exactly divergence-free. This is a
  // measurement, not a boundary condition.
  Grid3D mask(24, 16, 16, 2.4, 1.6, 1.6);
  voxelize_mesh(mask, fixtures::make_sphere({1.0, 0.8, 0.8}, 0.4, 16, 32));
  const auto faces = classify_faces(mask);
  MacGrid g(mask);

  g.set_uniform_velocity({1.0, 0.0, 0.0}, FaceSelection::All);
  EXPECT_EQ(max_abs_fluid(compute_divergence(g, mask), mask), 0.0);

  g.set_uniform_velocity({1.0, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  const auto d = compute_divergence(g, mask);
  std::size_t nonzero = 0;
  for (std::size_t k = 0; k < 16; ++k)
    for (std::size_t j = 0; j < 16; ++j)
      for (std::size_t i = 0; i < 24; ++i) {
        if (mask.cell(i, j, k) != CellType::Fluid) continue;
        if (unobstructed(faces, i, j, k)) {
          EXPECT_EQ(d.values(i, j, k), 0.0);
        } else if (d.values(i, j, k) != 0.0) {
          ++nonzero;
          EXPECT_DOUBLE_EQ(std::abs(d.values(i, j, k)), 1.0 / g.dx());  // one blocked x-face
        }
      }
  EXPECT_GT(nonzero, 0u);
}

TEST(DivergenceSolids, FullySolidDomainHasNoStatistics) {
  Grid3D mask(3, 3, 3, 1, 1, 1);
  mask.fill(CellType::Solid);
  MacGrid g(mask);
  g.set_uniform_velocity({1, 2, 3}, FaceSelection::All);
  const auto d = compute_divergence(g, mask);
  EXPECT_EQ(d.fluid_cells, 0u);
  EXPECT_EQ(count_fluid_cells(mask), 0u);
  for (double x : d.values.data()) EXPECT_EQ(x, 0.0);
  EXPECT_FALSE(divergence_stats(d, mask).has_value());
}

TEST(DivergenceSolids, FullyFluidDomainCountsEveryCell) {
  const Grid3D mask(3, 4, 5, 1, 1, 1);
  const auto d = compute_divergence(MacGrid(mask), mask);
  EXPECT_EQ(divergence_stats(d, mask)->fluid_cells, 60u);
}

// --- diagnostics ------------------------------------------------------------------------------

TEST(DivergenceStats, KnownValuesExcludeSolidCells) {
  // 3 x 1 x 1, dx = 1, middle cell solid. u faces 0, 1, 9, 6 -> fluid divergences 1 (cell 0) and -3 (cell 2).
  Grid3D mask(3, 1, 1, 3, 1, 1);
  mask.set_cell(1, 0, 0, CellType::Solid);
  MacGrid g(mask);
  g.u()(0, 0, 0) = 0.0;
  g.u()(1, 0, 0) = 1.0;
  g.u()(2, 0, 0) = 9.0;  // the solid cell's own divergence (8) must be ignored
  g.u()(3, 0, 0) = 6.0;
  const auto d = compute_divergence(g, mask);
  EXPECT_EQ(d.values(0, 0, 0), 1.0);
  EXPECT_EQ(d.values(1, 0, 0), 0.0);
  EXPECT_EQ(d.values(2, 0, 0), -3.0);
  const auto s = divergence_stats(d, mask);
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->fluid_cells, 2u);
  EXPECT_DOUBLE_EQ(s->max_abs, 3.0);
  EXPECT_DOUBLE_EQ(s->mean_abs, 2.0);
  EXPECT_DOUBLE_EQ(s->rms, std::sqrt(5.0));
}

TEST(DivergenceStats, MismatchedGridsAreRejected) {
  const Grid3D mask(4, 4, 4, 1, 1, 1);
  EXPECT_THROW((void)compute_divergence(MacGrid(4, 4, 5, 1, 1, 1), mask), std::invalid_argument);
  EXPECT_THROW((void)compute_divergence(MacGrid(4, 4, 4, 1, 1, 2), mask), std::invalid_argument);
  const auto d = compute_divergence(MacGrid(mask), mask);
  EXPECT_THROW((void)divergence_stats(d, Grid3D(4, 4, 3, 1, 1, 1)), std::invalid_argument);
}
