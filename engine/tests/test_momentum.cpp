#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

#include "aeroflow/box.hpp"
#include "aeroflow/momentum.hpp"

using namespace aeroflow;
using C = VelocityComponent;
using BC = VelocityFaceBC;

namespace {

template <typename F>
void fill_component(MacGrid& g, C c, F&& f) {
  StaggeredArray& a = c == C::U ? g.u() : c == C::V ? g.v() : g.w();
  for (std::size_t k = 0; k < a.n2(); ++k)
    for (std::size_t j = 0; j < a.n1(); ++j)
      for (std::size_t i = 0; i < a.n0(); ++i) {
        const Vec3 p = c == C::U ? g.u_position(i, j, k) : c == C::V ? g.v_position(i, j, k) : g.w_position(i, j, k);
        a(i, j, k) = f(p);
      }
}

}  // namespace

TEST(Momentum, UniformFieldHasNoTendency) {
  const Grid3D mask(10, 6, 5, 2.0, 0.9, 1.0);  // anisotropic
  const auto map = classify_boundaries(mask);
  MacGrid g(mask);
  g.set_uniform_velocity({5.0, 0.0, 0.0}, FaceSelection::All);
  apply_velocity_bcs(g, map, BoundaryConditions{5.0});
  for (std::size_t n = 0; n < map.u.size(); ++n) {
    if (map.u[n] != BC::Interior) continue;
    const std::size_t i = n % 11, j = (n / 11) % 6, k = n / 66;
    EXPECT_EQ(advection_term(g, map, C::U, i, j, k), 0.0);
    EXPECT_EQ(laplacian_term(g, map, C::U, i, j, k), 0.0);
  }
}

TEST(Momentum, LinearFieldIsAdvectedExactly) {
  // u = a x (v = w = 0): upwind is exact for linear profiles, -(u du/dx) = -a^2 x; lap u = 0.
  const Grid3D mask(12, 4, 4, 2.4, 0.8, 0.8);
  const auto map = classify_boundaries(mask);
  MacGrid g(mask);
  const double a = 1.5;
  fill_component(g, C::U, [&](const Vec3& p) { return a * p.x; });
  for (std::size_t i = 1; i < 12; ++i) {
    const double x = g.u_position(i, 1, 1).x;
    EXPECT_NEAR(advection_term(g, map, C::U, i, 1, 1), -a * a * x, 1e-12);
    EXPECT_NEAR(laplacian_term(g, map, C::U, i, 1, 1), 0.0, 1e-9);
  }
}

TEST(Momentum, QuadraticLaplacianOnAnisotropicSpacing) {
  // u = y^2 + 3 z^2 on interior faces away from walls: lap u = 2 + 6 exactly (centered stencil).
  const Grid3D mask(6, 8, 8, 1.2, 0.8, 1.6);
  const auto map = classify_boundaries(mask);
  MacGrid g(mask);
  fill_component(g, C::U, [](const Vec3& p) { return p.y * p.y + 3 * p.z * p.z; });
  for (std::size_t j = 1; j < 7; ++j)
    for (std::size_t k = 1; k < 7; ++k) EXPECT_NEAR(laplacian_term(g, map, C::U, 3, j, k), 8.0, 1e-9);
}

TEST(Momentum, UpwindingFollowsTheTransportDirection) {
  // w = step in x, transported by u = +-U: only the face downstream of the step changes.
  const Grid3D mask(10, 3, 3, 1.0, 0.3, 0.3);
  const auto map = classify_boundaries(mask);
  for (const double U : {+2.0, -2.0}) {
    MacGrid g(mask);
    g.set_uniform_velocity({U, 0.0, 0.0}, FaceSelection::All);
    fill_component(g, C::W, [](const Vec3& p) { return p.x < 0.5 ? 1e-6 : 0.0; });  // tiny: w dw/dz negligible
    // w faces: x = (i + 0.5) dx, the step lies between i = 4 (x = .45) and i = 5 (x = .55).
    const double at4 = advection_term(g, map, C::W, 4, 1, 1);
    const double at5 = advection_term(g, map, C::W, 5, 1, 1);
    if (U > 0) {
      EXPECT_NEAR(at4, 0.0, 1e-15);
      EXPECT_NEAR(at5, -U * (0.0 - 1e-6) / 0.1, 1e-12);  // = +2e-5: the step moves downstream
    } else {
      EXPECT_NEAR(at4, -U * (0.0 - 1e-6) / 0.1, 1e-12);
      EXPECT_NEAR(at5, 0.0, 1e-15);
    }
  }
}

TEST(Momentum, TransverseVelocityIsAveragedToTheFace) {
  // u = y (shear), v = c (uniform): -(vbar du/dy) = -c at interior u-faces; vbar must be c.
  const Grid3D mask(8, 6, 4, 1.6, 1.2, 0.8);
  const auto map = classify_boundaries(mask);
  MacGrid g(mask);
  fill_component(g, C::U, [](const Vec3& p) { return p.y; });
  fill_component(g, C::V, [](const Vec3&) { return 0.7; });
  // u du/dx = 0 (u independent of x), w = 0; v-faces on the y walls hold 0.7 too (not BC-applied).
  EXPECT_NEAR(advection_term(g, map, C::U, 4, 3, 2), -0.7 * 1.0, 1e-12);
  // Near the wall the upwind neighbor is a free-slip ghost (even): derivative 0 there.
  EXPECT_NEAR(advection_term(g, map, C::U, 4, 0, 2), 0.0, 1e-12);
}

TEST(Momentum, NeighborRulesMatchTheBoundaryModule) {
  Grid3D mask(12, 8, 8, 1.2, 0.8, 0.8);
  mark_solid_box(mask, {{0.4, 0.0, 0.2}, {0.7, 0.5, 0.6}});  // touches the floor
  const auto map = classify_boundaries(mask);
  MacGrid g(mask);
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> d(-1, 1);
  for (StaggeredArray* a : {&g.u(), &g.v(), &g.w()})
    for (double& x : a->data()) x = d(rng);
  std::size_t checked = 0;
  auto check = [&](C c, const StaggeredArray& a, const std::vector<BC>& bcs, int normal) {
    for (std::size_t k = 0; k < a.n2(); ++k)
      for (std::size_t j = 0; j < a.n1(); ++j)
        for (std::size_t i = 0; i < a.n0(); ++i) {
          if (bcs[a.index(i, j, k)] == BC::Inactive) continue;
          for (int axis = 0; axis < 3; ++axis) {
            if (axis == normal) continue;
            for (int side : {-1, 1}) {
              EXPECT_EQ(velocity_neighbor(g, map, c, i, j, k, axis, side),
                        tangential_neighbor(g, map, c, i, j, k, axis, side));
              ++checked;
            }
          }
        }
  };
  check(C::U, g.u(), map.u, 0);
  check(C::V, g.v(), map.v, 1);
  check(C::W, g.w(), map.w, 2);
  EXPECT_GT(checked, 1000u);
  // Along the own axis past the outlet: zero-gradient ghost.
  EXPECT_EQ(velocity_neighbor(g, map, C::U, 12, 4, 4, 0, +1), g.u()(12, 4, 4));
}

TEST(Momentum, OnlyInteriorFacesAreUpdated) {
  Grid3D mask(10, 6, 6, 1.0, 0.6, 0.6);
  mark_solid_box(mask, {{0.3, 0.2, 0.2}, {0.5, 0.4, 0.4}});
  const auto map = classify_boundaries(mask);
  MacGrid g(mask);
  g.set_uniform_velocity({5.0, 0.3, -0.2}, FaceSelection::All);
  StaggeredArray ou(11, 6, 6), ov(10, 7, 6), ow(10, 6, 7);
  explicit_momentum_update(g, map, MomentumTerms{true, true, 0.1}, 1e-3, ou, ov, ow);
  for (std::size_t n = 0; n < map.u.size(); ++n)
    if (map.u[n] != BC::Interior) EXPECT_EQ(ou.data()[n], g.u().data()[n]);
  for (std::size_t n = 0; n < map.v.size(); ++n)
    if (map.v[n] != BC::Interior) EXPECT_EQ(ov.data()[n], g.v().data()[n]);
  EXPECT_THROW((void)advection_term(g, map, C::U, 0, 2, 2), std::invalid_argument);  // inlet face
  StaggeredArray wrong(3, 3, 3);
  EXPECT_THROW(explicit_momentum_update(g, map, {}, 1e-3, wrong, ov, ow), std::invalid_argument);
}

TEST(Momentum, StabilityLimitFormula) {
  MacGrid g(4, 4, 4, 0.4, 0.8, 1.2);  // dx 0.1, dy 0.2, dz 0.3
  EXPECT_EQ(stability_limit(g, 0.0), std::numeric_limits<double>::infinity());
  g.u().fill(2.0);
  g.v().fill(-1.0);
  const double nu = 0.01;
  const double expected = 1.0 / (2.0 / 0.1 + 1.0 / 0.2 + 2 * nu * (100.0 + 25.0 + 1.0 / 0.09));
  EXPECT_DOUBLE_EQ(stability_limit(g, nu), expected);
}

TEST(Momentum, UpwindTransportConvergesAtFirstOrder) {
  // Transport of a smooth w(x) bump by uniform u = U with the advection operator alone (frozen
  // velocity, no projection). Amplitude is tiny so the nonlinear w dw/dz term is negligible.
  const double U = 1.0, T = 0.3, A = 1e-6, L = 2.0;
  auto bump = [&](double x) { return A * std::exp(-std::pow((x - 0.6) / 0.12, 2)); };
  std::printf("\n  %6s %14s %8s\n", "nx", "L1 error/A", "order");
  double prev = 0.0;
  for (const std::size_t nx : {50u, 100u, 200u, 400u}) {
    const Grid3D mask(nx, 1, 3, L, 0.1, 0.3);
    const auto map = classify_boundaries(mask);
    MacGrid g(mask);
    g.set_uniform_velocity({U, 0.0, 0.0}, FaceSelection::All);
    fill_component(g, C::W, [&](const Vec3& p) { return bump(p.x); });
    apply_prescribed_velocity_bcs(g, map, BoundaryConditions{U});
    StaggeredArray ou(g.u().n0(), 1, 3), ov(nx, 2, 3), ow(nx, 1, 4);
    const double dx = L / static_cast<double>(nx), dt = 0.5 * dx / U;
    const auto steps = static_cast<std::size_t>(std::llround(T / dt));
    for (std::size_t s = 0; s < steps; ++s) {
      explicit_momentum_update(g, map, MomentumTerms{true, false, 0.0}, dt, ou, ov, ow);
      std::swap(g.u(), ou);
      std::swap(g.v(), ov);
      std::swap(g.w(), ow);
    }
    double err = 0.0;
    for (std::size_t i = 0; i < nx; ++i) err += std::abs(g.w()(i, 0, 2) - bump(g.w_position(i, 0, 2).x - U * T)) * dx;
    err /= A;
    if (prev > 0) {
      const double order = std::log2(prev / err);
      std::printf("  %6zu %14.4e %8.3f\n", nx, err, order);
      EXPECT_NEAR(order, 1.0, 0.2);
    } else {
      std::printf("  %6zu %14.4e %8s\n", nx, err, "-");
    }
    prev = err;
  }
}
