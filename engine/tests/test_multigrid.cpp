#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "aeroflow/box.hpp"
#include "aeroflow/multigrid.hpp"
#include "aeroflow/voxelize.hpp"
#include "mesh_fixtures.hpp"

using namespace aeroflow;

namespace {

// Owns the grid, boundary map, pressure system and hierarchy with stable addresses.
struct Case {
  std::string name;
  std::unique_ptr<Grid3D> mask;
  std::unique_ptr<BoundaryMap> map;
  std::unique_ptr<PressureSystem> sys;
  std::unique_ptr<Multigrid> mg;
};

// Phase 2 tests exercise the trilinear transfers unless a test passes other settings.
MultigridSettings trilinear() {
  MultigridSettings s;
  s.transfer = MultigridTransfer::Trilinear;
  return s;
}

Case make_case(std::string name, Grid3D mask, MultigridSettings settings = trilinear()) {
  Case c;
  c.name = std::move(name);
  c.mask = std::make_unique<Grid3D>(std::move(mask));
  c.map = std::make_unique<BoundaryMap>(classify_boundaries(*c.mask));
  c.sys = std::make_unique<PressureSystem>(*c.mask, *c.map);
  c.mg = std::make_unique<Multigrid>(*c.mask, *c.map, *c.sys, settings);
  return c;
}

void solid_box(Grid3D& g, const Vec3& lo, const Vec3& hi) { mark_solid_box(g, {lo, hi}); }

// Odd counts in every direction, anisotropic spacing (dx 0.1, dy 0.05, dz 0.15), box obstacle.
Case odd_anisotropic_box() {
  Grid3D g(37, 13, 9, 3.7, 0.65, 1.35);
  solid_box(g, {1.0, 0.0, 0.3}, {1.5, 0.35, 0.9});
  return make_case("odd anisotropic box 37x13x9", std::move(g));
}

// Voxelized closed triangle mesh: the imported-STL code path (voxelize_mesh).
Case mesh_sphere() {
  Grid3D g(40, 20, 21, 4.0, 2.0, 2.1);
  voxelize_mesh(g, fixtures::make_sphere({1.4, 1.0, 1.05}, 0.5, 16, 32));
  return make_case("voxelized sphere mesh 40x20x21", std::move(g));
}

Case mesh_box() {
  Grid3D g(33, 17, 16, 3.3, 1.7, 1.6);
  voxelize_mesh(g, fixtures::make_box({0.83, 0.0, 0.41}, {1.57, 0.93, 1.13}));
  return make_case("voxelized box mesh 33x17x16", std::move(g));
}

// Two disconnected fluid regions besides the main flow: an enclosed cavity (hollow solid shell,
// Neumann everywhere, constant nullspace) and a one-cell pocket. A one-cell wall at an odd x index
// separates the cavity's two halves on the fine grid only; coarse cells straddle it.
Case disconnected() {
  Grid3D g(32, 16, 16, 3.2, 1.6, 1.6);
  solid_box(g, {1.0, 0.4, 0.4}, {2.0, 1.2, 1.2});  // shell
  for (std::size_t k = 0; k < 16; ++k)
    for (std::size_t j = 0; j < 16; ++j)
      for (std::size_t i = 0; i < 32; ++i) {
        const bool cavity = i >= 11 && i <= 18 && j >= 5 && j <= 10 && k >= 5 && k <= 10;
        if (cavity && i != 15) g.set_cell(i, j, k, CellType::Fluid);  // wall at i = 15 splits it in two
      }
  // Single-cell pocket in a solid block.
  solid_box(g, {2.4, 0.0, 0.0}, {2.7, 0.3, 0.3});
  g.set_cell(25, 1, 1, CellType::Fluid);
  return make_case("disconnected regions 32x16x16", std::move(g));
}

std::vector<Case> all_cases() {
  std::vector<Case> cases;
  cases.push_back(odd_anisotropic_box());
  cases.push_back(mesh_sphere());
  cases.push_back(mesh_box());
  cases.push_back(disconnected());
  return cases;
}

std::vector<double> random_vector(std::size_t n, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> d(-1.0, 1.0);
  std::vector<double> v(n);
  for (double& x : v) x = d(rng);
  return v;
}

double dot(const std::vector<double>& a, const std::vector<double>& b) {
  double s = 0.0;
  for (std::size_t n = 0; n < a.size(); ++n) s += a[n] * b[n];
  return s;
}

double norm(const std::vector<double>& a) { return std::sqrt(dot(a, a)); }

std::vector<double> apply_b(Multigrid& mg, const std::vector<double>& r) {
  std::vector<double> z(r.size());
  mg.apply(r, z);
  return z;
}

}  // namespace

TEST(Multigrid, SettingsAreValidated) {
  MultigridSettings s;
  EXPECT_NO_THROW(validate_multigrid_settings(s));
  s.post_sweeps = 1;
  EXPECT_THROW(validate_multigrid_settings(s), std::invalid_argument);  // asymmetric cycle
  s = {};
  s.coarse_sweeps = 0;
  EXPECT_THROW(validate_multigrid_settings(s), std::invalid_argument);
  s = {};
  s.semi_coarsening_ratio = 0.9;
  EXPECT_THROW(validate_multigrid_settings(s), std::invalid_argument);
}

TEST(Multigrid, FineLevelIsThePressureSystemOperatorBitForBit) {
  for (auto& c : all_cases()) {
    const std::size_t n = c.sys->unknowns();
    const auto x = random_vector(n, 1);
    std::vector<double> y_sys(n), y_mg(n);
    c.sys->apply(x, y_sys);
    c.mg->apply_operator(0, x, y_mg);
    for (std::size_t u = 0; u < n; ++u) ASSERT_EQ(y_mg[u], y_sys[u]) << c.name << " unknown " << u;
    EXPECT_EQ(c.mg->unknowns(0), n);
  }
}

TEST(Multigrid, HierarchyGeometryWithOddCountsAndSemiCoarsening) {
  // dx 0.1, dy 0.2, dz 0.1: y is coarser than 1.5 x the smallest width, so level 1 coarsens x and
  // z only; then all three are equal (0.2) and coarsen together. Odd counts give half cells.
  Grid3D g(41, 10, 21, 4.1, 2.0, 2.1);
  auto c = make_case("semi", std::move(g), MultigridSettings{2, 2, 16, 8, 16, 1.5, MultigridTransfer::Trilinear});
  ASSERT_GE(c.mg->levels(), 3u);
  EXPECT_EQ(c.mg->info(1).coarsened, (std::array<bool, 3>{true, false, true}));
  EXPECT_EQ(c.mg->info(1).dims, (std::array<std::size_t, 3>{21, 10, 11}));
  EXPECT_EQ(c.mg->info(2).coarsened, (std::array<bool, 3>{true, true, true}));
  EXPECT_EQ(c.mg->info(2).dims, (std::array<std::size_t, 3>{11, 5, 6}));
  const double L[3] = {4.1, 2.0, 2.1};
  for (std::size_t l = 0; l < c.mg->levels(); ++l)
    for (int a = 0; a < 3; ++a) {
      const auto w = c.mg->widths(l, a);
      const auto x = c.mg->centers(l, a);
      double sum = 0.0;
      for (std::size_t i = 0; i < w.size(); ++i) {
        EXPECT_NEAR(x[i], sum + 0.5 * w[i], 1e-12);  // true centre of every (partial) cell
        sum += w[i];
      }
      EXPECT_NEAR(sum, L[a], 1e-12) << "level " << l << " axis " << a;
    }
  // Level 1 along x: 20 full cells of 0.2 and a last half cell of 0.1.
  EXPECT_NEAR(c.mg->widths(1, 0)[19], 0.2, 1e-15);
  EXPECT_NEAR(c.mg->widths(1, 0)[20], 0.1, 1e-15);
}

TEST(Multigrid, CoarseConductancesUseTrueAreasAndCentreDistances) {
  // Empty tunnel, h = 0.1, nx = 5 (odd): level 1 has x widths 0.2, 0.2, 0.1 (centres 0.1, 0.3, 0.45).
  Grid3D g(5, 4, 4, 0.5, 0.4, 0.4);
  auto c = make_case("empty", std::move(g), MultigridSettings{2, 2, 16, 1, 16, 1.5, MultigridTransfer::Trilinear});
  ASSERT_GE(c.mg->levels(), 2u);
  const double area = 0.2 * 0.2;  // a coarse face spans 2 x 2 fully open fine faces
  EXPECT_NEAR(c.mg->conductance(1, 0, 1, 0, 0), area / 0.2, 1e-12);          // full | full
  EXPECT_NEAR(c.mg->conductance(1, 0, 2, 1, 1), area / 0.15, 1e-12);         // full | half cell
  EXPECT_NEAR(c.mg->conductance(1, 0, 3, 0, 1), area / (0.5 * 0.1), 1e-12);  // outlet: half width
  EXPECT_EQ(c.mg->conductance(1, 0, 0, 0, 0), 0.0);                          // inlet: Neumann
  EXPECT_EQ(c.mg->conductance(1, 1, 0, 0, 0), 0.0);                          // floor: Neumann
  EXPECT_EQ(c.mg->conductance(1, 1, 0, 2, 0), 0.0);                          // ceiling: Neumann
  EXPECT_NEAR(c.mg->conductance(1, 1, 0, 1, 0), (0.2 * 0.2) / 0.2, 1e-12);   // y face: dx(0.2) x dz(0.2)
  // Rediscretization agrees with the fine operator for the first level (K_0 = V_0 A):
  // a fully open fine face has T = dy dz / dx = 0.1, i.e. A's 1/dx^2 times V_0 = 1e-3.
  const double v0 = 1e-3;
  EXPECT_NEAR(c.sys->inverse_spacing_squared()[0] * v0, 0.1 * 0.1 / 0.1, 1e-15);
}

TEST(Multigrid, CoarseOperatorsAreSymmetricPositiveSemidefiniteAndConsistent) {
  for (auto& c : all_cases()) {
    for (std::size_t l = 1; l < c.mg->levels(); ++l) {
      const std::size_t n = c.mg->unknowns(l);
      std::vector<double> Kx(n), Ky(n), ones(n, 1.0), K1(n);
      for (unsigned seed = 0; seed < 5; ++seed) {
        const auto x = random_vector(n, 10 + seed), y = random_vector(n, 20 + seed);
        c.mg->apply_operator(l, x, Kx);
        c.mg->apply_operator(l, y, Ky);
        const double scale = norm(Kx) * norm(y) + norm(Ky) * norm(x);
        EXPECT_LE(std::abs(dot(Kx, y) - dot(x, Ky)), 1e-13 * scale) << c.name << " level " << l;
        EXPECT_GE(dot(x, Kx), -1e-13 * norm(Kx) * norm(x)) << c.name << " level " << l;
      }
      // Rows sum to the outlet conductance only: K 1 >= 0, with K 1 = 0 away from the outlet.
      c.mg->apply_operator(l, ones, K1);
      for (std::size_t u = 0; u < n; ++u) EXPECT_GE(K1[u], -1e-12) << c.name << " level " << l;
      EXPECT_GT(c.mg->info(l).components, 0u);
    }
  }
}

TEST(Multigrid, RestrictionIsTheExactTransposeOfProlongation) {
  // Inner product: the Euclidean dot product over unknowns used by solve_pressure (PCG).
  for (auto& c : all_cases()) {
    for (std::size_t l = 0; l + 1 < c.mg->levels(); ++l) {
      const std::size_t nf = c.mg->unknowns(l), nc = c.mg->unknowns(l + 1);
      const auto xf = random_vector(nf, 3), yc = random_vector(nc, 4);
      std::vector<double> Py(nf), Rx(nc);
      c.mg->prolong(l, yc, Py);
      c.mg->restrict_transpose(l, xf, Rx);
      const double lhs = dot(Rx, yc), rhs = dot(xf, Py);
      EXPECT_LE(std::abs(lhs - rhs), 1e-13 * norm(xf) * norm(Py) * std::sqrt(8.0)) << c.name << " level " << l;
      // Every row of P sums to one: constants are reproduced exactly.
      std::vector<double> ones(nc, 1.0), P1(nf);
      c.mg->prolong(l, ones, P1);
      for (std::size_t u = 0; u < nf; ++u) ASSERT_NEAR(P1[u], 1.0, 1e-14) << c.name << " level " << l;
      // Trilinear: a field linear in x is reproduced away from boundaries and obstacles (checked on
      // the empty-interior cells of the box case below).
    }
  }
}

TEST(Multigrid, ProlongationIsTrilinearInTheInterior) {
  Grid3D g(16, 8, 8, 1.6, 0.8, 0.8);
  auto c = make_case("empty", std::move(g), MultigridSettings{2, 2, 16, 1, 16, 1.5, MultigridTransfer::Trilinear});
  const std::size_t nc = c.mg->unknowns(1), nf = c.mg->unknowns(0);
  std::vector<double> coarse(nc), fine(nf);
  for (std::size_t K = 0; K < 4; ++K)
    for (std::size_t J = 0; J < 4; ++J)
      for (std::size_t I = 0; I < 8; ++I) {
        const double x = c.mg->centers(1, 0)[I], y = c.mg->centers(1, 1)[J], z = c.mg->centers(1, 2)[K];
        coarse[static_cast<std::size_t>(c.mg->unknown_of_cell(1, I, J, K))] = 1.0 + 2.0 * x - 3.0 * y + 0.5 * z;
      }
  c.mg->prolong(0, coarse, fine);
  for (std::size_t k = 1; k < 7; ++k)
    for (std::size_t j = 1; j < 7; ++j)
      for (std::size_t i = 1; i < 15; ++i) {
        const double x = (static_cast<double>(i) + 0.5) * 0.1, y = (static_cast<double>(j) + 0.5) * 0.1,
                     z = (static_cast<double>(k) + 0.5) * 0.1;
        EXPECT_NEAR(fine[static_cast<std::size_t>(c.mg->unknown_of_cell(0, i, j, k))], 1.0 + 2.0 * x - 3.0 * y + 0.5 * z,
                    1e-12);
      }
}

TEST(Multigrid, VcycleIsLinearSymmetricAndPositiveDefinite) {
  for (auto& c : all_cases()) {
    const std::size_t n = c.sys->unknowns();
    double min_rayleigh = std::numeric_limits<double>::infinity();
    for (unsigned seed = 0; seed < 6; ++seed) {
      const auto x = random_vector(n, 100 + seed), y = random_vector(n, 200 + seed);
      const auto Bx = apply_b(*c.mg, x), By = apply_b(*c.mg, y);
      // Linear (fixed work, no data-dependent decisions).
      std::vector<double> combo(n);
      for (std::size_t u = 0; u < n; ++u) combo[u] = 2.5 * x[u] - y[u];
      const auto Bc = apply_b(*c.mg, combo);
      double lin = 0.0;
      for (std::size_t u = 0; u < n; ++u) lin = std::max(lin, std::abs(Bc[u] - (2.5 * Bx[u] - By[u])));
      EXPECT_LE(lin, 1e-12 * (2.5 * norm(Bx) + norm(By))) << c.name;
      // Symmetric in the PCG inner product.
      const double scale = norm(Bx) * norm(y) + norm(By) * norm(x);
      EXPECT_LE(std::abs(dot(Bx, y) - dot(x, By)), 1e-12 * scale) << c.name;
      // Positive definite.
      const double rq = dot(x, Bx) / dot(x, x);
      EXPECT_GT(rq, 0.0) << c.name;
      min_rayleigh = std::min(min_rayleigh, rq);
    }
    std::printf("  %-36s levels %zu, min x^T B x / x^T x over random x = %.3e\n", c.name.c_str(), c.mg->levels(),
                min_rayleigh);
  }
}

TEST(Multigrid, DisconnectedEnclosedAndIsolatedRegions) {
  auto c = disconnected();
  const auto& fine = c.mg->info(0);
  // Main flow, the two cavity halves (split by the one-cell wall) and the pocket.
  EXPECT_EQ(fine.components, 4u);
  EXPECT_EQ(fine.enclosed_components, 3u);
  EXPECT_EQ(fine.isolated_cells, 1u);
  // Face-opening aggregation keeps an axis-aligned one-cell wall closed on level 1 (the coarse face
  // over it has no open area), so level 1 has the same regions. On level 2 the coarse cells are
  // wide enough to contain fluid from both sides of the wall and from the pocket's neighbourhood:
  // regions merge (a coarse-grid topology change), which must not break symmetry or definiteness.
  ASSERT_GE(c.mg->levels(), 3u);
  EXPECT_EQ(c.mg->info(1).components, fine.components);
  EXPECT_LT(c.mg->info(2).components, fine.components);
  for (std::size_t l = 0; l < c.mg->levels(); ++l) {
    const auto& i = c.mg->info(l);
    std::printf("  level %zu: %zux%zux%zu, %zu unknowns, %zu components (%zu enclosed), %zu isolated\n", l, i.dims[0],
                i.dims[1], i.dims[2], i.unknowns, i.components, i.enclosed_components, i.isolated_cells);
  }

  // B is positive on the nullspace directions of A as well (constants on enclosed regions, the
  // pocket's unit vector), not only on its range.
  const std::size_t n = c.sys->unknowns();
  std::vector<std::vector<double>> null_vectors;
  for (std::size_t comp = 0; comp < c.sys->components().size(); ++comp) {
    if (c.sys->components()[comp].has_outlet) continue;
    std::vector<double> v(n, 0.0);
    for (std::size_t u = 0; u < n; ++u) v[u] = c.sys->component_of(u) == comp ? 1.0 : 0.0;
    null_vectors.push_back(v);
  }
  ASSERT_EQ(null_vectors.size(), 3u);
  for (const auto& v : null_vectors) {
    std::vector<double> Av(n);
    c.sys->apply(v, Av);
    EXPECT_LE(norm(Av), 1e-12);  // indeed in the nullspace of A
    const auto Bv = apply_b(*c.mg, v);
    EXPECT_GT(dot(v, Bv), 0.0);
  }

  // Prolongation never mixes regions that are separate on the coarse level: a coarse vector that
  // is constant on one coarse component maps to a fine vector supported only on fine cells whose
  // parents belong to that component.
  const std::size_t nc = c.mg->unknowns(1);
  std::vector<double> coarse(nc, 0.0), fine_v(n);
  const std::uint32_t target = c.mg->component_of(1, 0);
  for (std::size_t u = 0; u < nc; ++u) coarse[u] = c.mg->component_of(1, u) == target ? 1.0 : 0.0;
  c.mg->prolong(0, coarse, fine_v);
  for (double v : fine_v) EXPECT_TRUE(v == 0.0 || std::abs(v - 1.0) < 1e-14);
}

TEST(Multigrid, OutputsStayFiniteAndScaleLinearly) {
  auto c = odd_anisotropic_box();
  const std::size_t n = c.sys->unknowns();
  const auto zero = apply_b(*c.mg, std::vector<double>(n, 0.0));
  for (double v : zero) EXPECT_EQ(v, 0.0);
  const auto r1 = random_vector(n, 7);
  const auto z1 = apply_b(*c.mg, r1);
  EXPECT_GT(dot(r1, z1), 0.0);
  for (const double s : {1e-200, 1e-8, 1e8, 1e200}) {
    auto r = r1;
    for (double& v : r) v *= s;
    const auto z = apply_b(*c.mg, r);
    for (std::size_t u = 0; u < n; ++u) {
      ASSERT_TRUE(std::isfinite(z[u])) << "scale " << s;
      ASSERT_NEAR(z[u] / s, z1[u], 1e-12 * norm(z1)) << "scale " << s;  // no overflow or underflow
    }
  }
}

TEST(Multigrid, SingleLevelAndTinyGridsAreHandled) {
  // Fewer unknowns than the coarsest threshold: the cycle is the fixed symmetric smoother alone.
  Grid3D g(6, 3, 3, 0.6, 0.3, 0.3);
  solid_box(g, {0.2, 0.0, 0.0}, {0.3, 0.1, 0.1});
  auto c = make_case("tiny", std::move(g));
  EXPECT_EQ(c.mg->levels(), 1u);
  const std::size_t n = c.sys->unknowns();
  const auto x = random_vector(n, 1), y = random_vector(n, 2);
  const auto Bx = apply_b(*c.mg, x), By = apply_b(*c.mg, y);
  EXPECT_LE(std::abs(dot(Bx, y) - dot(x, By)), 1e-12 * (norm(Bx) * norm(y) + norm(By) * norm(x)));
  EXPECT_GT(dot(x, Bx), 0.0);
  // A 1-cell-wide axis cannot be coarsened; the others still are.
  Grid3D flat(64, 32, 1, 6.4, 3.2, 0.1);
  auto f = make_case("flat", std::move(flat));
  ASSERT_GE(f.mg->levels(), 2u);
  EXPECT_FALSE(f.mg->info(1).coarsened[2]);
  EXPECT_EQ(f.mg->info(1).dims[2], 1u);
}

TEST(Multigrid, PreconditionsConjugateGradientsInATestOnlyLoop) {
  // Not the production solver (phase 3 wires MG into solve_pressure). This only checks that B is a
  // usable SPD preconditioner: PCG with B reaches the same true residual as the baseline. No timing.
  Grid3D g(64, 32, 32, 4.0, 2.0, 2.0);
  solid_box(g, {1.0, 0.0, 0.75}, {1.5, 0.5, 1.25});
  auto c = make_case("box 64x32x32", std::move(g));
  const std::size_t n = c.sys->unknowns();
  // Right-hand side: -div of a perturbed uniform field.
  MacGrid mac(*c.mask);
  const auto faces = classify_faces(*c.mask);
  mac.set_uniform_velocity({5.0, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  apply_velocity_bcs(mac, *c.map, BoundaryConditions{5.0});
  std::vector<double> b(n);
  c.sys->build_rhs(mac, b);
  const double tol = 1e-10, bnorm = norm(b);
  ASSERT_GT(bnorm, 0.0);

  auto pcg = [&](const std::function<void(const std::vector<double>&, std::vector<double>&)>& precond) {
    std::vector<double> x(n, 0.0), r = b, z(n), d(n), Ad(n);
    precond(r, z);
    d = z;
    double rz = dot(r, z);
    std::size_t it = 0;
    while (norm(r) > tol * bnorm && it < 2000) {
      c.sys->apply(d, Ad);
      const double alpha = rz / dot(d, Ad);
      for (std::size_t u = 0; u < n; ++u) {
        x[u] += alpha * d[u];
        r[u] -= alpha * Ad[u];
      }
      precond(r, z);
      const double rz_new = dot(r, z);
      for (std::size_t u = 0; u < n; ++u) d[u] = z[u] + (rz_new / rz) * d[u];
      rz = rz_new;
      ++it;
    }
    c.sys->apply(x, Ad);
    for (std::size_t u = 0; u < n; ++u) r[u] = b[u] - Ad[u];
    return std::make_pair(it, norm(r) / bnorm);
  };
  const auto inv = c.sys->inverse_diagonal();
  const auto jacobi = pcg([&](const std::vector<double>& r, std::vector<double>& z) {
    for (std::size_t u = 0; u < n; ++u) z[u] = inv[u] * r[u];
  });
  const auto mg = pcg([&](const std::vector<double>& r, std::vector<double>& z) { c.mg->apply(r, z); });
  EXPECT_LE(jacobi.second, 2 * tol);
  EXPECT_LE(mg.second, 2 * tol);
  EXPECT_LT(mg.first, jacobi.first);
  std::printf("\n  test-only PCG, box 64x32x32, tol 1e-10: Jacobi %zu iterations (true rel. residual %.2e), "
              "V(2,2) %zu iterations (%.2e), %zu levels\n",
              jacobi.first, jacobi.second, mg.first, mg.second, c.mg->levels());
}

TEST(Multigrid, DenseVcycleMatrixIsSymmetricAndHasACholeskyFactor) {
  // Small enough to assemble B column by column: odd anisotropic counts, an obstacle, an enclosed
  // cavity and an isolated cell, three levels. Cholesky succeeds iff the symmetric part is SPD.
  Grid3D g(13, 9, 7, 1.3, 0.45, 1.05);  // dx 0.1, dy 0.05, dz 0.15
  solid_box(g, {0.3, 0.0, 0.15}, {0.6, 0.25, 0.6});
  solid_box(g, {0.8, 0.1, 0.3}, {1.2, 0.4, 0.9});
  g.set_cell(9, 4, 3, CellType::Fluid);  // enclosed cavity cells (two, face-connected)
  g.set_cell(10, 4, 3, CellType::Fluid);
  solid_box(g, {0.0, 0.35, 0.75}, {0.3, 0.45, 1.05});
  g.set_cell(1, 8, 6, CellType::Fluid);  // isolated pocket in the top corner
  auto c = make_case("dense", std::move(g), MultigridSettings{2, 2, 16, 4, 16, 1.5, MultigridTransfer::Trilinear});
  ASSERT_GE(c.mg->levels(), 3u);
  EXPECT_GE(c.mg->info(0).enclosed_components, 2u);
  EXPECT_EQ(c.mg->info(0).isolated_cells, 1u);
  const std::size_t n = c.sys->unknowns();
  std::vector<double> B(n * n), e(n, 0.0), col(n);
  for (std::size_t j = 0; j < n; ++j) {
    e[j] = 1.0;
    c.mg->apply(e, col);
    e[j] = 0.0;
    for (std::size_t i = 0; i < n; ++i) B[i * n + j] = col[i];
  }
  double asym = 0.0, maxabs = 0.0;
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t j = 0; j < n; ++j) {
      asym = std::max(asym, std::abs(B[i * n + j] - B[j * n + i]));
      maxabs = std::max(maxabs, std::abs(B[i * n + j]));
    }
  EXPECT_LE(asym, 1e-13 * maxabs);
  // Cholesky of the symmetric part.
  std::vector<double> L(n * n, 0.0);
  double min_pivot = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < n; ++j) {
    double d = 0.5 * (B[j * n + j] + B[j * n + j]);
    for (std::size_t k = 0; k < j; ++k) d -= L[j * n + k] * L[j * n + k];
    ASSERT_GT(d, 0.0) << "pivot " << j;
    min_pivot = std::min(min_pivot, d);
    L[j * n + j] = std::sqrt(d);
    for (std::size_t i = j + 1; i < n; ++i) {
      double s = 0.5 * (B[i * n + j] + B[j * n + i]);
      for (std::size_t k = 0; k < j; ++k) s -= L[i * n + k] * L[j * n + k];
      L[i * n + j] = s / L[j * n + j];
    }
  }
  std::printf("  dense B (%zu unknowns, %zu levels): max asymmetry %.2e of max |B_ij| %.2e, min Cholesky pivot %.3e\n", n,
              c.mg->levels(), asym, maxabs, min_pivot);
}

TEST(Multigrid, CoarseOperatorMatchesTheGalerkinProductInTheInterior) {
  // For a quadratic field, trilinear prolongation errs by a constant that K_0 annihilates, so in
  // the interior V_0 P^T A P phi must equal the rediscretized K_1 phi exactly. This checks the
  // coarse scaling (V_0, R = P^T, conductance form) end to end, with full and semi-coarsening.
  struct Spec {
    std::array<std::size_t, 3> n;
    std::array<double, 3> L;
    std::array<bool, 3> coarsened;
  };
  const Spec specs[] = {
      {{16, 16, 16}, {1.6, 1.6, 1.6}, {true, true, true}},   // isotropic: all axes
      {{16, 12, 8}, {1.6, 0.6, 0.8}, {false, true, false}},  // dy = dx/2: only y (the finest)
  };
  for (const auto& sp : specs) {
    Grid3D g(sp.n[0], sp.n[1], sp.n[2], sp.L[0], sp.L[1], sp.L[2]);
    const double v0 = g.dx() * g.dy() * g.dz();
    auto c = make_case("galerkin", std::move(g), MultigridSettings{2, 2, 16, 1, 16, 1.5, MultigridTransfer::Trilinear});
    ASSERT_GE(c.mg->levels(), 2u);
    EXPECT_EQ(c.mg->info(1).coarsened, sp.coarsened);
    const std::size_t nc = c.mg->unknowns(1), nf = c.mg->unknowns(0);
    const auto dims = c.mg->info(1).dims;
    std::vector<double> phi(nc), Pphi(nf), APphi(nf), galerkin(nc), K1phi(nc);
    for (std::size_t K = 0; K < dims[2]; ++K)
      for (std::size_t J = 0; J < dims[1]; ++J)
        for (std::size_t I = 0; I < dims[0]; ++I) {
          const double x = c.mg->centers(1, 0)[I], y = c.mg->centers(1, 1)[J], z = c.mg->centers(1, 2)[K];
          phi[static_cast<std::size_t>(c.mg->unknown_of_cell(1, I, J, K))] = x * x - 2 * y * y + 3 * z * z + x * y;
        }
    c.mg->prolong(0, phi, Pphi);
    c.sys->apply(Pphi, APphi);
    c.mg->restrict_transpose(0, APphi, galerkin);
    for (double& v : galerkin) v *= v0;
    c.mg->apply_operator(1, phi, K1phi);
    double worst = 0.0, scale = 0.0;
    for (std::size_t K = 2; K + 2 < dims[2]; ++K)
      for (std::size_t J = 2; J + 2 < dims[1]; ++J)
        for (std::size_t I = 2; I + 2 < dims[0]; ++I) {
          const auto u = static_cast<std::size_t>(c.mg->unknown_of_cell(1, I, J, K));
          worst = std::max(worst, std::abs(galerkin[u] - K1phi[u]));
          scale = std::max(scale, std::abs(K1phi[u]));
        }
    ASSERT_GT(scale, 0.0);
    EXPECT_LE(worst, 1e-10 * scale);
    std::printf("  %zux%zux%zu: interior |V0 P^T A P phi - K1 phi| / |K1 phi| = %.2e\n", sp.n[0], sp.n[1], sp.n[2],
                worst / scale);
  }
}

TEST(Multigrid, CachedTransfersEqualTheReferenceStencil) {
  for (auto& c : all_cases()) {
    for (std::size_t l = 0; l + 1 < c.mg->levels(); ++l) {
      const std::size_t nf = c.mg->unknowns(l), nc = c.mg->unknowns(l + 1);
      const auto xc = random_vector(nc, 9);
      std::vector<double> fast(nf), ref(nf);
      c.mg->prolong(l, xc, fast);
      c.mg->prolong_reference(l, xc, ref);
      double worst = 0.0;
      std::size_t at = 0;
      for (std::size_t u = 0; u < nf; ++u)
        if (std::abs(fast[u] - ref[u]) > worst) {
          worst = std::abs(fast[u] - ref[u]);
          at = u;
        }
      EXPECT_LE(worst, 1e-14) << c.name << " level " << l << " unknown " << at << " fast " << fast[at] << " ref " << ref[at];
    }
  }
}

TEST(Multigrid, ConstantTransfersKeepTheCycleSymmetricPositiveDefinite) {
  MultigridSettings constant;
  constant.transfer = MultigridTransfer::Constant;
  std::vector<Case> cases;
  {
    Grid3D g(37, 13, 9, 3.7, 0.65, 1.35);
    solid_box(g, {1.0, 0.0, 0.3}, {1.5, 0.35, 0.9});
    cases.push_back(make_case("odd anisotropic box", std::move(g), constant));
  }
  {
    Grid3D g(40, 20, 21, 4.0, 2.0, 2.1);
    voxelize_mesh(g, fixtures::make_sphere({1.4, 1.0, 1.05}, 0.5, 16, 32));
    cases.push_back(make_case("voxelized sphere", std::move(g), constant));
  }
  {
    auto d = disconnected();
    cases.push_back(make_case("disconnected", Grid3D(*d.mask), constant));
  }
  for (auto& c : cases) {
    for (std::size_t l = 0; l + 1 < c.mg->levels(); ++l) {
      const std::size_t nf = c.mg->unknowns(l), nc = c.mg->unknowns(l + 1);
      const auto xf = random_vector(nf, 3), yc = random_vector(nc, 4);
      std::vector<double> Py(nf), Rx(nc), ref(nf), ones(nc, 1.0), P1(nf);
      c.mg->prolong(l, yc, Py);
      c.mg->prolong_reference(l, yc, ref);
      c.mg->restrict_transpose(l, xf, Rx);
      EXPECT_LE(std::abs(dot(Rx, yc) - dot(xf, Py)), 1e-13 * norm(xf) * norm(Py) * std::sqrt(8.0)) << c.name;
      for (std::size_t u = 0; u < nf; ++u) ASSERT_EQ(Py[u], ref[u]) << c.name;  // P picks the parent value
      c.mg->prolong(l, ones, P1);
      for (double v : P1) ASSERT_EQ(v, 1.0);
    }
    const std::size_t n = c.sys->unknowns();
    for (unsigned seed = 0; seed < 4; ++seed) {
      const auto x = random_vector(n, 50 + seed), y = random_vector(n, 60 + seed);
      const auto Bx = apply_b(*c.mg, x), By = apply_b(*c.mg, y);
      EXPECT_LE(std::abs(dot(Bx, y) - dot(x, By)), 1e-12 * (norm(Bx) * norm(y) + norm(By) * norm(x))) << c.name;
      EXPECT_GT(dot(x, Bx), 0.0) << c.name;
    }
  }
  // Dense check with an obstacle, a cavity and an isolated cell (as for trilinear transfers).
  Grid3D g(13, 9, 7, 1.3, 0.45, 1.05);
  solid_box(g, {0.3, 0.0, 0.15}, {0.6, 0.25, 0.6});
  solid_box(g, {0.8, 0.1, 0.3}, {1.2, 0.4, 0.9});
  g.set_cell(9, 4, 3, CellType::Fluid);
  g.set_cell(10, 4, 3, CellType::Fluid);
  solid_box(g, {0.0, 0.35, 0.75}, {0.3, 0.45, 1.05});
  g.set_cell(1, 8, 6, CellType::Fluid);
  MultigridSettings s = constant;
  s.coarsest_unknowns = 4;
  auto c = make_case("dense constant", std::move(g), s);
  const std::size_t n = c.sys->unknowns();
  std::vector<double> B(n * n), e(n, 0.0), col(n);
  for (std::size_t j = 0; j < n; ++j) {
    e[j] = 1.0;
    c.mg->apply(e, col);
    e[j] = 0.0;
    for (std::size_t i = 0; i < n; ++i) B[i * n + j] = col[i];
  }
  double asym = 0.0;
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t j = 0; j < n; ++j) asym = std::max(asym, std::abs(B[i * n + j] - B[j * n + i]));
  EXPECT_LE(asym, 1e-13);
  std::vector<double> L(n * n, 0.0);
  for (std::size_t j = 0; j < n; ++j) {
    double d = B[j * n + j];
    for (std::size_t k = 0; k < j; ++k) d -= L[j * n + k] * L[j * n + k];
    ASSERT_GT(d, 0.0) << "pivot " << j;
    L[j * n + j] = std::sqrt(d);
    for (std::size_t i = j + 1; i < n; ++i) {
      double v = 0.5 * (B[i * n + j] + B[j * n + i]);
      for (std::size_t k = 0; k < j; ++k) v -= L[i * n + k] * L[j * n + k];
      L[i * n + j] = v / L[j * n + j];
    }
  }
}
