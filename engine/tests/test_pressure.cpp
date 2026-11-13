#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <random>
#include <vector>

#include "aeroflow/box.hpp"
#include "aeroflow/config.hpp"
#include "aeroflow/field_export.hpp"
#include "aeroflow/field_io.hpp"
#include "aeroflow/pressure.hpp"
#include "aeroflow/voxelize.hpp"
#include "mesh_fixtures.hpp"

using namespace aeroflow;
using BC = VelocityFaceBC;

namespace {

const BoundaryConditions kBC{5.0};

std::vector<double> snapshot(const MacGrid& mac) {
  std::vector<double> out;
  for (const StaggeredArray* a : {&mac.u(), &mac.v(), &mac.w()}) out.insert(out.end(), a->data().begin(), a->data().end());
  return out;
}

std::vector<BC> roles(const BoundaryMap& map) {
  std::vector<BC> out(map.u);
  out.insert(out.end(), map.v.begin(), map.v.end());
  out.insert(out.end(), map.w.begin(), map.w.end());
  return out;
}

// u* = U_in on active faces (the initialized field), perturbed on interior faces.
void initialize(MacGrid& mac, const Grid3D& mask, double perturb, unsigned seed = 3) {
  const auto faces = classify_faces(mask);
  mac.set_uniform_velocity({5.0, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  if (perturb == 0.0) return;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> d(-perturb, perturb);
  for (StaggeredArray* a : {&mac.u(), &mac.v(), &mac.w()})
    for (double& x : a->data()) x += d(rng);
}

double vel_scale_div(const MacGrid& mac) { return 5.0 / std::min({mac.dx(), mac.dy(), mac.dz()}); }

Grid3D obstacle_grid() {
  Grid3D mask(20, 10, 12, 2.0, 0.75, 1.2);  // dx 0.1, dy 0.075, dz 0.1 (anisotropic)
  mark_solid_box(mask, {{0.6, 0.15, 0.3}, {1.0, 0.6, 0.8}});
  return mask;
}

}  // namespace

// --- 7. operator consistency ------------------------------------------------------------------

TEST(PressureOperator, LaplacianEqualsDivergenceOfTheProjectionGradient) {
  // A phi must equal D(-G phi): apply the velocity correction to a zero field and take D.
  const auto mask = obstacle_grid();
  const auto map = classify_boundaries(mask);
  const PressureSystem sys(mask, map);
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> d(-1, 1);
  std::vector<double> phi(sys.unknowns()), Aphi(sys.unknowns());
  for (double& x : phi) x = d(rng);
  sys.apply(phi, Aphi);

  MacGrid mac(mask);  // all zero
  sys.correct_velocity(mac, phi);  // u = -G phi on coupled/outlet faces
  const auto div = compute_divergence(mac, mask);
  double max_err = 0.0, scale = 0.0;
  for (std::size_t n = 0; n < sys.unknowns(); ++n) {
    max_err = std::max(max_err, std::abs(div.values.data()[sys.cell_of_unknown(n)] - Aphi[n]));
    scale = std::max(scale, std::abs(Aphi[n]));
  }
  EXPECT_LE(max_err, 1e-12 * scale);
}

TEST(PressureOperator, IsSymmetricAndPositiveDefinite) {
  const auto mask = obstacle_grid();
  const auto map = classify_boundaries(mask);
  const PressureSystem sys(mask, map);
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> d(-1, 1);
  std::vector<double> x(sys.unknowns()), y(sys.unknowns()), Ax(sys.unknowns()), Ay(sys.unknowns());
  for (std::size_t n = 0; n < x.size(); ++n) {
    x[n] = d(rng);
    y[n] = d(rng);
  }
  sys.apply(x, Ax);
  sys.apply(y, Ay);
  double xAy = 0, yAx = 0, xAx = 0;
  for (std::size_t n = 0; n < x.size(); ++n) {
    xAy += x[n] * Ay[n];
    yAx += y[n] * Ax[n];
    xAx += x[n] * Ax[n];
  }
  EXPECT_NEAR(xAy, yAx, 1e-10 * std::abs(xAy));
  EXPECT_GT(xAx, 0.0);
  // Constants are not in the nullspace: the outlet anchors the pressure.
  std::vector<double> ones(sys.unknowns(), 1.0), Aones(sys.unknowns());
  sys.apply(ones, Aones);
  double s = 0;
  for (double v : Aones) s += v;
  EXPECT_GT(s, 0.0);
  EXPECT_EQ(sys.components().size(), 1u);
  EXPECT_TRUE(sys.components()[0].has_outlet);
}

// --- 6. half-cell outlet ----------------------------------------------------------------------

TEST(PressureOperator, OutletDirichletSitsOnTheOutletPlane) {
  // 1D: A phi = -a with Neumann at x = 0 and phi = 0 on the plane x = Lx. The continuous solution
  // is phi = a (x^2 - Lx^2)/2; the discrete solution is that quadratic shifted by -a dx^2/8
  // (half-cell Dirichlet), i.e. second-order accurate. Pinning phi = 0 at the last cell center
  // instead would be first order (error a Lx dx / 2).
  const double a = 3.0, Lx = 2.0;
  std::vector<double> errors;
  for (const std::size_t nx : {8u, 16u, 32u, 64u}) {
    const Grid3D mask(nx, 1, 1, Lx, 0.3, 0.2);
    const auto map = classify_boundaries(mask);
    const PressureSystem sys(mask, map);
    const double dx = Lx / static_cast<double>(nx);

    // (1) The closed-form discrete solution satisfies the system to rounding.
    std::vector<double> expected(nx), A_expected(nx);
    for (std::size_t i = 0; i < nx; ++i) {
      const double x = (static_cast<double>(i) + 0.5) * dx;
      expected[i] = a * (x * x - Lx * Lx) / 2 - a * dx * dx / 8;
    }
    sys.apply(expected, A_expected);
    for (std::size_t i = 0; i < nx; ++i) EXPECT_NEAR(A_expected[i], -a, 1e-10);

    // (2) The solver reproduces it (tolerance near the rounding floor of this 1D system).
    std::vector<double> b(nx, -a), phi(nx);
    ProjectionSettings s;
    s.tolerance = 1e-11;
    solve_pressure(sys, b, phi, s);
    double max_err = 0.0;
    for (std::size_t i = 0; i < nx; ++i) {
      EXPECT_NEAR(phi[i], expected[i], 1e-8);
      const double x = (static_cast<double>(i) + 0.5) * dx;
      max_err = std::max(max_err, std::abs(expected[i] - a * (x * x - Lx * Lx) / 2));
    }
    errors.push_back(max_err);
  }
  // Second order: halving dx divides the error by 4.
  for (std::size_t n = 1; n < errors.size(); ++n) EXPECT_NEAR(errors[n - 1] / errors[n], 4.0, 1e-9);
}

// --- 1. uniform flow ----------------------------------------------------------------------------

TEST(Projection, UniformFlowInAnEmptyTunnelIsUnchanged) {
  const Grid3D mask(16, 8, 6, 2.0, 0.6, 0.9);
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  initialize(mac, mask, 0.0);
  apply_velocity_bcs(mac, map, kBC);
  const auto before = snapshot(mac);
  const auto res = project_velocity(mac, mask, map, kBC, {});
  EXPECT_EQ(res.solve.iterations, 0u);
  EXPECT_TRUE(res.solve.converged);
  EXPECT_EQ(snapshot(mac), before);  // bit for bit
  for (double p : mac.p().data()) EXPECT_EQ(p, 0.0);
  EXPECT_EQ(res.after_stats->max_abs, 0.0);
  EXPECT_DOUBLE_EQ(res.fluxes.outlet, res.fluxes.inlet);
}

// --- 2. divergent field ---------------------------------------------------------------------

TEST(Projection, RemovesDivergenceDownToTheSolverResidual) {
  const Grid3D mask(24, 12, 10, 2.4, 0.9, 1.5);  // anisotropic
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  initialize(mac, mask, 2.0);
  const auto res = project_velocity(mac, mask, map, kBC, {});
  ASSERT_TRUE(res.solve.converged);
  const double before = res.before_stats->rms, after = res.after_stats->rms;
  std::printf("\n  divergent field: RMS div %.3e -> %.3e (%zu iterations, rel. residual %.2e)\n", before, after,
              res.solve.iterations, res.solve.relative_residual);
  EXPECT_GT(before, 1.0);
  EXPECT_LE(res.after_stats->max_abs, 1e-8 * vel_scale_div(mac));
  EXPECT_LE(after, 10 * ProjectionSettings{}.tolerance * before);
  EXPECT_LE(res.solve.relative_residual, ProjectionSettings{}.tolerance);
}

TEST(Projection, RemainingDivergenceEqualsTheNegatedResidual) {
  const auto mask = obstacle_grid();
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  initialize(mac, mask, 1.0);
  apply_velocity_bcs(mac, map, kBC);
  const PressureSystem sys(mask, map);
  std::vector<double> b(sys.unknowns()), phi(sys.unknowns()), Aphi(sys.unknowns());
  sys.build_rhs(mac, b);
  ProjectionSettings loose;
  loose.tolerance = 1e-4;  // stop early so the residual is far from zero
  solve_pressure(sys, b, phi, loose);
  sys.apply(phi, Aphi);
  sys.correct_velocity(mac, phi);
  const auto div = compute_divergence(mac, mask);
  double max_err = 0.0, max_r = 0.0;
  for (std::size_t n = 0; n < sys.unknowns(); ++n) {
    const double r = b[n] - Aphi[n];
    max_err = std::max(max_err, std::abs(div.values.data()[sys.cell_of_unknown(n)] + r));
    max_r = std::max(max_r, std::abs(r));
  }
  EXPECT_GT(max_r, 1e-8);
  EXPECT_LE(max_err, 1e-12 * vel_scale_div(mac));
}

// --- 3/4. obstacles, prescribed faces and fluxes --------------------------------------------------

TEST(Projection, ObstacleNoPenetrationAndPrescribedFacesUntouched) {
  Grid3D mask(40, 20, 20, 4.0, 2.0, 2.0);
  voxelize_mesh(mask, fixtures::make_sphere({1.4, 1.0, 1.0}, 0.5, 16, 32));
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  initialize(mac, mask, 0.0);
  apply_velocity_bcs(mac, map, kBC);
  EXPECT_EQ(boundary_fluxes(mac, map).wall_leak, 0.0);
  const auto before = snapshot(mac);
  const auto res = project_velocity(mac, mask, map, kBC, {});
  const auto after = snapshot(mac);
  const auto r = roles(map);
  std::size_t prescribed = 0, changed = 0;
  for (std::size_t n = 0; n < r.size(); ++n) {
    if (is_prescribed(r[n])) {
      EXPECT_EQ(after[n], before[n]);
      ++prescribed;
    } else if (after[n] != before[n]) {
      ++changed;
    }
  }
  EXPECT_GT(prescribed, 0u);
  EXPECT_GT(changed, 0u);  // the flow is deflected around the obstacle
  EXPECT_EQ(res.fluxes.wall_leak, 0.0);
  EXPECT_LE(std::abs(res.fluxes.outlet - res.fluxes.inlet), 1e-8 * res.fluxes.inlet);
  EXPECT_LE(res.after_stats->max_abs, 1e-8 * vel_scale_div(mac));
  EXPECT_GT(res.before_stats->max_abs, 10.0);  // the initialized field violates continuity
  std::printf("\n  sphere obstacle 40x20x20: %zu iterations, max|div| %.2e -> %.2e, Q_in %.6f Q_out %.6f\n",
              res.solve.iterations, res.before_stats->max_abs, res.after_stats->max_abs, res.fluxes.inlet,
              res.fluxes.outlet);
}

TEST(Projection, PressureScalesWithDensityOverTimestep) {
  const auto mask = obstacle_grid();
  const auto map = classify_boundaries(mask);
  MacGrid a(mask), b(mask);
  initialize(a, mask, 0.0);
  initialize(b, mask, 0.0);
  ProjectionSettings s1, s2;
  s2.density = 2 * s1.density;
  s2.dt = 0.5 * projection_timestep(s1, a, 5.0);
  const auto r1 = project_velocity(a, mask, map, kBC, s1);
  project_velocity(b, mask, map, kBC, s2);
  EXPECT_DOUBLE_EQ(r1.dt, 0.5 * 0.075 / 5.0);  // CFL default: 0.5 min(h) / U_in
  for (std::size_t n = 0; n < a.p().size(); ++n) EXPECT_NEAR(b.p().data()[n], 4 * a.p().data()[n], 1e-9 * (1 + std::abs(b.p().data()[n])));
  EXPECT_EQ(snapshot(a), snapshot(b));  // velocity does not depend on rho or dt
}

TEST(Projection, ApplyingTwiceChangesNothingFurther) {
  const auto mask = obstacle_grid();
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  initialize(mac, mask, 1.0);
  ProjectionSettings s;
  s.tolerance = 1e-12;
  project_velocity(mac, mask, map, kBC, s);
  const auto once = snapshot(mac);
  // Second projection: the outlet predictor copy is re-applied by apply_velocity_bcs, so the
  // field may shift slightly, but it stays divergence-free and prescribed faces are unchanged.
  const auto res = project_velocity(mac, mask, map, kBC, s);
  EXPECT_LE(res.after_stats->max_abs, 1e-8 * vel_scale_div(mac));
  const auto twice = snapshot(mac);
  const auto r = roles(map);
  for (std::size_t n = 0; n < r.size(); ++n)
    if (is_prescribed(r[n])) EXPECT_EQ(twice[n], once[n]);
}

// --- 8. convergence -----------------------------------------------------------------------------

TEST(PressureSolver, ConvergesAndReportsHistory) {
  std::printf("\n  %10s %8s %12s %10s\n", "grid", "iters", "rel.resid", "solve ms");
  std::size_t prev_iters = 0;
  for (const std::size_t n : {8u, 16u, 32u}) {
    Grid3D mask(2 * n, n, n, 2.0, 1.0, 1.0);
    mark_solid_box(mask, {{0.6, 0.3, 0.3}, {0.9, 0.7, 0.7}});
    const auto map = classify_boundaries(mask);
    MacGrid mac(mask);
    initialize(mac, mask, 0.0);
    ProjectionSettings jacobi;
    jacobi.solver = PressureSolverKind::JacobiPcg;  // the Milestone 5 baseline (auto would pick MG-PCG)
    const auto res = project_velocity(mac, mask, map, kBC, jacobi);
    std::printf("  %4zux%zux%zu %8zu %12.3e %10.2f\n", 2 * n, n, n, res.solve.iterations,
                res.solve.relative_residual, res.solve_ms);
    EXPECT_TRUE(res.solve.converged);
    EXPECT_EQ(res.solve.history.size(), res.solve.iterations);
    EXPECT_LE(res.solve.history.back(), 1e-10);
    EXPECT_GT(res.solve.iterations, prev_iters);  // harder with refinement (Jacobi PCG)
    prev_iters = res.solve.iterations;
  }
}

TEST(PressureSolver, NonConvergenceIsAnError) {
  const auto mask = obstacle_grid();
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  initialize(mac, mask, 1.0);
  ProjectionSettings s;
  s.max_iterations = 3;
  try {
    project_velocity(mac, mask, map, kBC, s);
    FAIL() << "expected ProjectionError";
  } catch (const ProjectionError& e) {
    EXPECT_EQ(e.code(), "projection_not_converged");
    EXPECT_NE(std::string(e.what()).find("after 3 iterations"), std::string::npos);
  }
}

// --- 9. invalid settings ----------------------------------------------------------------------

TEST(PressureSettings, RejectsInvalidValues) {
  auto expect_invalid = [](ProjectionSettings s) {
    try {
      validate_projection_settings(s);
      FAIL() << "expected invalid_settings";
    } catch (const ProjectionError& e) {
      EXPECT_EQ(e.code(), "invalid_settings");
    }
  };
  for (double rho : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    ProjectionSettings s;
    s.density = rho;
    expect_invalid(s);
  }
  for (double dt : {0.0, -1e-3, std::numeric_limits<double>::infinity()}) {
    ProjectionSettings s;
    s.dt = dt;
    expect_invalid(s);
  }
  for (double tol : {0.0, 1.0, 2.0, -1e-9}) {
    ProjectionSettings s;
    s.tolerance = tol;
    expect_invalid(s);
  }
  ProjectionSettings s;
  s.max_iterations = 0;
  expect_invalid(s);
  // dt is required when there is no inflow speed to build the CFL default from.
  const MacGrid g(4, 4, 4, 1, 1, 1);
  EXPECT_THROW((void)projection_timestep(ProjectionSettings{}, g, 0.0), ProjectionError);
  ProjectionSettings explicit_dt;
  explicit_dt.dt = 1e-3;
  EXPECT_EQ(projection_timestep(explicit_dt, g, 0.0), 1e-3);
}

// --- 10. disconnected regions -------------------------------------------------------------------

TEST(PressureComponents, EnclosedCavityIsSolvedWithZeroMeanGauge) {
  // A hollow solid shell with a fluid cavity inside: the cavity has no inlet or outlet.
  Grid3D mask(16, 12, 12, 1.6, 1.2, 1.2);
  mark_solid_box(mask, {{0.4, 0.3, 0.3}, {1.2, 0.9, 0.9}});
  for (std::size_t k = 4; k < 8; ++k)
    for (std::size_t j = 4; j < 8; ++j)
      for (std::size_t i = 6; i < 10; ++i) mask.set_cell(i, j, k, CellType::Fluid);
  const auto map = classify_boundaries(mask);
  const PressureSystem sys(mask, map);
  ASSERT_EQ(sys.components().size(), 2u);
  MacGrid mac(mask);
  initialize(mac, mask, 1.0);  // divergent inside the cavity too
  const auto res = project_velocity(mac, mask, map, kBC, {});
  EXPECT_EQ(res.enclosed_components, 1u);
  EXPECT_LE(res.after_stats->max_abs, 1e-8 * vel_scale_div(mac));
  double mean = 0.0;
  std::size_t count = 0;
  for (std::size_t k = 4; k < 8; ++k)
    for (std::size_t j = 4; j < 8; ++j)
      for (std::size_t i = 6; i < 10; ++i) {
        mean += mac.p()(i, j, k);
        ++count;
      }
  EXPECT_NEAR(mean / static_cast<double>(count), 0.0, 1e-6);
}

TEST(PressureComponents, InletRegionWithoutOutletIsRejected) {
  // A solid plate spanning the whole cross-section: the upstream region has inflow and no exit.
  Grid3D mask(12, 6, 6, 1.2, 0.6, 0.6);
  mark_solid_box(mask, {{0.6, 0.0, 0.0}, {0.7, 0.6, 0.6}});
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  initialize(mac, mask, 0.0);
  try {
    project_velocity(mac, mask, map, kBC, {});
    FAIL() << "expected incompatible_component";
  } catch (const ProjectionError& e) {
    EXPECT_EQ(e.code(), "incompatible_component");
    EXPECT_NE(std::string(e.what()).find("fed by the inlet"), std::string::npos);
  }
  // With no inflow the same region is compatible and projects fine.
  MacGrid still(mask);
  ProjectionSettings s;
  s.dt = 1e-3;
  const auto res = project_velocity(still, mask, map, BoundaryConditions{0.0}, s);
  EXPECT_EQ(res.enclosed_components, 1u);
}

TEST(PressureComponents, SingleCellPocketIsHandled) {
  Grid3D mask(5, 5, 5, 1, 1, 1);
  mark_solid_box(mask, {{0.2, 0.2, 0.2}, {0.8, 0.8, 0.8}});
  mask.set_cell(2, 2, 2, CellType::Fluid);  // isolated: all six faces SolidWall
  const auto map = classify_boundaries(mask);
  MacGrid mac(mask);
  initialize(mac, mask, 0.0);
  const auto res = project_velocity(mac, mask, map, kBC, {});
  EXPECT_EQ(res.enclosed_components, 1u);
  EXPECT_EQ(mac.p()(2, 2, 2), 0.0);
}

// --- velocity magnitude -------------------------------------------------------------------------

// --- warm start across timestep changes (Milestone 7, phase 1) -----------------------------------

namespace {

// u* = dt * f on Interior faces, prescribed faces 0 (U_in = 0): b = -D u* is exactly proportional
// to dt, so the potential is too (phi = dt * psi) while the pressure p = rho phi / dt is not.
MacGrid scaled_field(const Grid3D& mask, const BoundaryMap& map, double dt) {
  MacGrid mac(mask);
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> d(-1.0, 1.0);
  auto fill = [&](StaggeredArray& a, const std::vector<BC>& bcs) {
    for (std::size_t n = 0; n < a.size(); ++n) a.data()[n] = bcs[n] == BC::Interior ? dt * d(rng) : 0.0;
  };
  fill(mac.u(), map.u);
  fill(mac.v(), map.v);
  fill(mac.w(), map.w);
  return mac;
}

}  // namespace

TEST(Projector, WarmStartCarriesThePressureAcrossTimestepChanges) {
  const auto mask = obstacle_grid();
  const auto map = classify_boundaries(mask);
  const BoundaryConditions bc{0.0};
  ProjectionSettings settings;
  settings.tolerance = 1e-10;
  settings.solver = PressureSolverKind::JacobiPcg;  // phase 1 baseline; MG-PCG: test_mgpcg.cpp
  Projector projector(mask, map);
  const double dt_full = 2e-3, dt_short = 0.05 * dt_full;

  // Full step, cold.
  MacGrid full = scaled_field(mask, map, dt_full);
  const auto r1 = projector.project(full, bc, settings, dt_full);
  EXPECT_FALSE(r1.warm_started);
  EXPECT_GT(r1.solve.iterations, 10u);
  const std::vector<double> p_full(full.p().data().begin(), full.p().data().end());

  // Shortened step with the same pressure: the rescaled guess (dt_short/dt_full) phi already meets
  // the tolerance, so no iterations are needed and the pressure is unchanged.
  MacGrid shortened = scaled_field(mask, map, dt_short);
  const auto r2 = projector.project(shortened, bc, settings, dt_short);
  EXPECT_TRUE(r2.warm_started);
  EXPECT_EQ(r2.solve.iterations, 0u);
  EXPECT_LE(r2.solve.relative_residual, settings.tolerance);
  double pmax = 0.0, dp = 0.0;
  for (std::size_t n = 0; n < p_full.size(); ++n) {
    pmax = std::max(pmax, std::abs(p_full[n]));
    dp = std::max(dp, std::abs(shortened.p().data()[n] - p_full[n]));
  }
  EXPECT_LE(dp, 1e-12 * pmax);

  // The following full step is rescaled back up (x 20) and again needs no iterations.
  MacGrid next = scaled_field(mask, map, dt_full);
  const auto r3 = projector.project(next, bc, settings, dt_full);
  EXPECT_TRUE(r3.warm_started);
  EXPECT_EQ(r3.solve.iterations, 0u);
  EXPECT_LE(r3.solve.relative_residual, settings.tolerance);

  // The previous behaviour (reusing phi without rescaling) starts from a residual ~19 ||b|| and
  // needs more iterations than a cold start.
  const PressureSystem sys(mask, map);
  MacGrid probe = scaled_field(mask, map, dt_short);
  apply_velocity_bcs(probe, map, bc);
  std::vector<double> b(sys.unknowns()), phi_full(sys.unknowns()), cold(sys.unknowns());
  {
    MacGrid f = scaled_field(mask, map, dt_full);
    apply_velocity_bcs(f, map, bc);
    sys.build_rhs(f, b);
    (void)solve_pressure(sys, b, phi_full, settings);
  }
  sys.build_rhs(probe, b);
  PcgWorkspace ws;
  const auto cold_stats = solve_pressure(sys, b, cold, settings, ws, {false, false});
  std::vector<double> unscaled = phi_full;
  const auto unscaled_stats = solve_pressure(sys, b, unscaled, settings, ws, {true, false});
  EXPECT_GT(unscaled_stats.iterations, cold_stats.iterations);
  std::printf("\n  shortened step (0.05 dt): cold %zu, unscaled warm %zu, rescaled warm %zu PCG iterations\n",
              cold_stats.iterations, unscaled_stats.iterations, r2.solve.iterations);

  // reset_warm_start(): the next solve is cold and agrees with the warm one to the tolerance.
  projector.reset_warm_start();
  MacGrid again = scaled_field(mask, map, dt_full);
  const auto r4 = projector.project(again, bc, settings, dt_full);
  EXPECT_FALSE(r4.warm_started);
  EXPECT_EQ(r4.solve.iterations, r1.solve.iterations);
  for (std::size_t n = 0; n < p_full.size(); ++n) EXPECT_NEAR(again.p().data()[n], p_full[n], 1e-8 * pmax);
}

TEST(Projection, VelocityMagnitudeIsCellCentred) {
  Grid3D mask(6, 4, 4, 1.2, 0.8, 0.8);
  mask.set_cell(2, 1, 1, CellType::Solid);
  MacGrid mac(mask);
  mac.set_uniform_velocity({3.0, 4.0, 0.0}, FaceSelection::All);
  const auto mag = velocity_magnitude(mac, mask);
  EXPECT_DOUBLE_EQ(mag(0, 0, 0), 5.0);
  EXPECT_EQ(mag(2, 1, 1), 0.0);  // solid: invalid
  mac.u()(1, 0, 0) = 1.0;        // faces of cell 0: u = (3 + 1)/2 = 2
  EXPECT_DOUBLE_EQ(velocity_magnitude(mac, mask)(0, 0, 0), std::sqrt(4.0 + 16.0));
}

// --- AFFIELD export of projection fields ------------------------------------------------------

TEST(ProjectionFields, ExportsPressureDivergenceAndSpeedFromOneProjection) {
  const auto dir = std::filesystem::temp_directory_path() / "aeroflow_projection_fields";
  std::filesystem::create_directories(dir);
  const auto mask = obstacle_grid();
  const auto out = export_fields(mask, {kPressure, kDivergenceBefore, kDivergenceAfter, kVelocityMagnitude},
                                 dir.string(), ProjectionConfig{});
  ASSERT_EQ(out.fields.size(), 4u);
  ASSERT_FALSE(out.projection.is_null());
  EXPECT_EQ(out.projection["converged"], true);
  EXPECT_EQ(out.projection["dt_source"], "cfl_0.5");
  EXPECT_GT(out.projection["iterations"].get<int>(), 0);
  EXPECT_EQ(out.projection["fluxes"]["wall_leak"], 0.0);

  const auto p = read_field_file((dir / "pressure.f0.affield").string());
  EXPECT_EQ(p.header["units"], "Pa");
  EXPECT_EQ(p.header["signed"], true);
  EXPECT_EQ(p.header["provenance"]["kind"], "projection");
  EXPECT_EQ(p.header["provenance"]["solved"], false);
  const auto before = read_field_file((dir / "divergence_before.f0.affield").string());
  const auto after = read_field_file((dir / "divergence_after.f0.affield").string());
  EXPECT_GT(before.header["stats"]["max_abs"].get<double>(), 10.0);
  EXPECT_LE(after.header["stats"]["max_abs"].get<double>(), 1e-8 * 5.0 / 0.075);
  const auto speed = read_field_file((dir / "velocity_magnitude.f0.affield").string());
  EXPECT_EQ(speed.header["signed"], false);
  EXPECT_EQ(speed.header["units"], "m/s");
  EXPECT_GT(speed.header["stats"]["max"].get<double>(), 5.0);  // flow accelerates around the obstacle
  // Solid cells are invalid in every projection field.
  for (std::size_t c = 0; c < mask.cell_count(); ++c)
    EXPECT_EQ(p.valid[c], mask.cells()[c] == CellType::Fluid ? 1 : 0);
  std::filesystem::remove_all(dir);
}

TEST(ProjectionFields, DiagnosticFieldIsUnchangedByDefault) {
  const auto f = divergence_diagnostic(obstacle_grid());
  EXPECT_NE(f.provenance["description"].get<std::string>().find("uniform +x 5 m/s"), std::string::npos);
}

TEST(ProjectionFields, ConfigBlockIsValidated) {
  auto base = nlohmann::json::parse(R"({"domain": {"lx": 1, "ly": 1, "lz": 1}, "resolution": {"nx": 2, "ny": 2, "nz": 2}})");
  auto with = [&](nlohmann::json projection) {
    auto j = base;
    j["projection"] = std::move(projection);
    return j;
  };
  const auto ok = parse_config(with({{"inlet_velocity", 7.5}, {"density", 1.0}, {"dt", 0.002},
                                     {"tolerance", 1e-8}, {"max_iterations", 500}}));
  ASSERT_TRUE(ok.projection.has_value());
  EXPECT_EQ(ok.projection->inlet_velocity, 7.5);
  EXPECT_EQ(*ok.projection->dt, 0.002);
  EXPECT_EQ(ok.projection->max_iterations, 500u);
  EXPECT_FALSE(parse_config(with({{"dt", nullptr}})).projection->dt.has_value());
  for (const auto& bad : {nlohmann::json{{"density", 0}}, nlohmann::json{{"dt", -1}}, nlohmann::json{{"tolerance", 1}},
                          nlohmann::json{{"max_iterations", 0}}, nlohmann::json{{"inlet_velocity", -2}},
                          nlohmann::json{{"rho", 1}}}) {
    EXPECT_THROW((void)parse_config(with(bad)), ConfigError) << bad.dump();
  }
}
