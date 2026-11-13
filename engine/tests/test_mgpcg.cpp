// Milestone 7, phase 3: multigrid-preconditioned CG in the production pressure solver.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "aeroflow/box.hpp"
#include "aeroflow/config.hpp"
#include "aeroflow/multigrid.hpp"
#include "aeroflow/pressure.hpp"
#include "aeroflow/simulation.hpp"
#include "aeroflow/voxelize.hpp"
#include "mesh_fixtures.hpp"

using namespace aeroflow;
using BC = VelocityFaceBC;

namespace {

constexpr double kU = 5.0;

struct Geometry {
  std::string name;
  Grid3D mask;
  double perturb;  // random interior perturbation of u* (0 where enclosed regions must stay compatible)
};

// Fluid regions: main flow; an enclosed cavity split by a one-cell wall into two Neumann regions
// (they merge on multigrid level 2); and an isolated single cell.
Grid3D disconnected_mask() {
  Grid3D g(32, 16, 16, 3.2, 1.6, 1.6);
  mark_solid_box(g, {{1.0, 0.4, 0.4}, {2.0, 1.2, 1.2}});
  for (std::size_t k = 5; k <= 10; ++k)
    for (std::size_t j = 5; j <= 10; ++j)
      for (std::size_t i = 11; i <= 18; ++i)
        if (i != 15) g.set_cell(i, j, k, CellType::Fluid);
  mark_solid_box(g, {{2.4, 0.0, 0.0}, {2.7, 0.3, 0.3}});
  g.set_cell(25, 1, 1, CellType::Fluid);
  return g;
}

std::vector<Geometry> geometries() {
  std::vector<Geometry> out;
  {
    Grid3D g(37, 13, 9, 3.7, 0.65, 1.35);  // odd counts, dx 0.1, dy 0.05, dz 0.15
    mark_solid_box(g, {{1.0, 0.0, 0.3}, {1.5, 0.35, 0.9}});
    out.push_back({"odd anisotropic box 37x13x9", std::move(g), 0.5});
  }
  {
    Grid3D g(40, 20, 21, 4.0, 2.0, 2.1);
    voxelize_mesh(g, fixtures::make_sphere({1.4, 1.0, 1.05}, 0.5, 16, 32));
    out.push_back({"voxelized sphere mesh 40x20x21", std::move(g), 0.5});
  }
  {
    Grid3D g(48, 16, 12, 2.4, 2.4, 1.2);  // dx 0.05, dy 0.15, dz 0.1: strong anisotropy (semi-coarsening)
    mark_solid_box(g, {{0.6, 0.0, 0.3}, {0.9, 1.2, 0.8}});
    out.push_back({"strongly anisotropic box 48x16x12", std::move(g), 0.5});
  }
  out.push_back({"disconnected regions 32x16x16", disconnected_mask(), 0.0});
  return out;
}

void initialize(MacGrid& mac, const Grid3D& mask, double perturb, const BoundaryMap& map) {
  const auto faces = classify_faces(mask);
  mac.set_uniform_velocity({kU, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  if (perturb == 0.0) return;
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> d(-perturb, perturb);
  auto add = [&](StaggeredArray& a, const std::vector<BC>& roles) {
    for (std::size_t n = 0; n < a.size(); ++n)
      if (roles[n] == BC::Interior) a.data()[n] += d(rng);
  };
  add(mac.u(), map.u);
  add(mac.v(), map.v);
  add(mac.w(), map.w);
}

std::vector<double> velocities(const MacGrid& g) {
  std::vector<double> out;
  for (const StaggeredArray* a : {&g.u(), &g.v(), &g.w()}) out.insert(out.end(), a->data().begin(), a->data().end());
  return out;
}

double max_abs_diff(const std::vector<double>& a, const std::vector<double>& b) {
  double m = 0.0;
  for (std::size_t n = 0; n < a.size(); ++n) m = std::max(m, std::abs(a[n] - b[n]));
  return m;
}

double norm(std::span<const double> v) {
  double s = 0.0;
  for (double x : v) s += x * x;
  return std::sqrt(s);
}

ProjectionSettings settings_for(PressureSolverKind kind, double tol = 1e-10) {
  ProjectionSettings s;
  s.dt = 1e-3;
  s.tolerance = tol;
  s.solver = kind;
  return s;
}

}  // namespace

TEST(Mgpcg, SolverNamesAreStable) {
  EXPECT_EQ(to_string(PressureSolverKind::JacobiPcg), "jacobi_pcg");
  EXPECT_EQ(to_string(PressureSolverKind::Mgpcg), "mgpcg");
  EXPECT_EQ(to_string(PressureSolverKind::Auto), "auto");
  for (const auto k : {PressureSolverKind::Auto, PressureSolverKind::Mgpcg, PressureSolverKind::JacobiPcg})
    EXPECT_EQ(parse_pressure_solver(to_string(k)), k);
  EXPECT_FALSE(parse_pressure_solver("multigrid").has_value());
  EXPECT_EQ(ProjectionSettings{}.solver, PressureSolverKind::Auto);  // phase 5 default
  EXPECT_EQ(SimulationSettings{}.solver, PressureSolverKind::Auto);
  EXPECT_EQ(MultigridSettings{}.transfer, MultigridTransfer::Constant);    // phase 4 measured default
}

TEST(Mgpcg, AutoSelectsBySizeAndReportsTheChoice) {
  EXPECT_EQ(resolve_solver(PressureSolverKind::Auto, kAutoMultigridMinUnknowns - 1), PressureSolverKind::JacobiPcg);
  EXPECT_EQ(resolve_solver(PressureSolverKind::Auto, kAutoMultigridMinUnknowns), PressureSolverKind::Mgpcg);
  EXPECT_EQ(resolve_solver(PressureSolverKind::JacobiPcg, 1u << 20), PressureSolverKind::JacobiPcg);
  EXPECT_EQ(resolve_solver(PressureSolverKind::Mgpcg, 10), PressureSolverKind::Mgpcg);
  ProjectionSettings autos;
  autos.dt = 1e-3;
  for (const std::size_t nx : {8u, 32u}) {  // 8x4x4 = 128 cells, 32x16x16 = 8192 cells
    Grid3D mask(nx, nx / 2, nx / 2, 1.0, 0.5, 0.5);
    const auto map = classify_boundaries(mask);
    MacGrid mac(mask);
    initialize(mac, mask, 0.3, map);
    const auto r = project_velocity(mac, mask, map, BoundaryConditions{kU}, autos);
    EXPECT_EQ(r.solve.requested, PressureSolverKind::Auto);
    const auto expected = nx == 8 ? PressureSolverKind::JacobiPcg : PressureSolverKind::Mgpcg;
    EXPECT_EQ(r.solve.selected, expected);
    EXPECT_EQ(r.solve.used, expected);
    EXPECT_EQ(r.solve.multigrid_levels > 0, expected == PressureSolverKind::Mgpcg);
    EXPECT_TRUE(r.solve.converged);
  }
}

TEST(Mgpcg, ProjectsLikeJacobiPcgOnEveryGeometry) {
  // Both solves meet ||b - A phi|| <= tol ||b||, so A (phi_mg - phi_j) = r_j - r_mg is bounded by
  // 2 tol ||b||: the projected velocities differ only at the level of the solver tolerance.
  for (auto& geo : geometries()) {
    const auto map = classify_boundaries(geo.mask);
    MacGrid base(geo.mask);
    initialize(base, geo.mask, geo.perturb, map);
    MacGrid jac = base, mg = base;
    const auto rj = project_velocity(jac, geo.mask, map, BoundaryConditions{kU}, settings_for(PressureSolverKind::JacobiPcg));
    const auto rm = project_velocity(mg, geo.mask, map, BoundaryConditions{kU}, settings_for(PressureSolverKind::Mgpcg));
    SCOPED_TRACE(geo.name);

    // Requested and used solver, no fallback, a real hierarchy.
    EXPECT_EQ(rj.solve.used, PressureSolverKind::JacobiPcg);
    EXPECT_EQ(rm.solve.requested, PressureSolverKind::Mgpcg);
    EXPECT_EQ(rm.solve.used, PressureSolverKind::Mgpcg);
    EXPECT_FALSE(rm.solve.fallback) << rm.solve.fallback_reason;
    EXPECT_GE(rm.solve.multigrid_levels, 2u);
    EXPECT_GT(rm.solve.setup_ms, 0.0);
    EXPECT_TRUE(rj.solve.converged);
    EXPECT_TRUE(rm.solve.converged);
    EXPECT_LE(rm.solve.relative_residual, 1e-10);
    EXPECT_LT(rm.solve.iterations, rj.solve.iterations);

    // Same pressure system: A (phi_mg - phi_j) is within the two residuals.
    const PressureSystem sys(geo.mask, map);
    const double scale = 1e-3 / 1.225;  // phi = (dt/rho) p
    std::vector<double> dphi(sys.unknowns()), Ad(sys.unknowns());
    for (std::size_t u = 0; u < sys.unknowns(); ++u) {
      const std::size_t c = sys.cell_of_unknown(u);
      dphi[u] = scale * (mg.p().data()[c] - jac.p().data()[c]);
    }
    sys.apply(dphi, Ad);
    EXPECT_LE(norm(Ad), 2.0 * 1e-10 * rj.solve.rhs_norm * (1 + 1e-6));

    // Projected velocities, divergence, mass balance and impermeability.
    const double dv = max_abs_diff(velocities(mg), velocities(jac));
    EXPECT_LE(dv, 1e-6 * kU);
    EXPECT_LE(rm.after_stats->max_abs, 10 * std::max(rj.after_stats->max_abs, 1e-12));
    EXPECT_EQ(rm.fluxes.wall_leak, 0.0);
    EXPECT_EQ(rj.fluxes.wall_leak, 0.0);
    EXPECT_NEAR(rm.fluxes.outlet, rm.fluxes.inlet, 1e-8 * rm.fluxes.inlet);
    EXPECT_NEAR(rm.fluxes.outlet, rj.fluxes.outlet, 1e-9 * rm.fluxes.inlet);
    // Prescribed faces (inlet, walls, solids) are identical bit for bit.
    for (std::size_t n = 0; n < map.u.size(); ++n)
      if (map.u[n] != BC::Interior && map.u[n] != BC::Outlet) ASSERT_EQ(mg.u().data()[n], jac.u().data()[n]);
    for (std::size_t n = 0; n < map.v.size(); ++n)
      if (map.v[n] != BC::Interior) ASSERT_EQ(mg.v().data()[n], jac.v().data()[n]);
    for (std::size_t n = 0; n < map.w.size(); ++n)
      if (map.w[n] != BC::Interior) ASSERT_EQ(mg.w().data()[n], jac.w().data()[n]);

    // Enclosed regions: same zero-mean gauge with both solvers.
    std::vector<double> mean_mg(sys.components().size(), 0.0), mean_j(sys.components().size(), 0.0);
    for (std::size_t u = 0; u < sys.unknowns(); ++u) {
      const std::size_t c = sys.cell_of_unknown(u);
      mean_mg[sys.component_of(u)] += mg.p().data()[c];
      mean_j[sys.component_of(u)] += jac.p().data()[c];
    }
    for (std::size_t c = 0; c < sys.components().size(); ++c) {
      if (sys.components()[c].has_outlet) continue;
      EXPECT_NEAR(mean_mg[c], 0.0, 1e-9);
      EXPECT_NEAR(mean_j[c], 0.0, 1e-9);
    }
    std::printf("  %-36s Jacobi %4zu it, MGPCG %3zu it (%zu levels); max |du| %.2e m/s, div_max %.2e / %.2e\n",
                geo.name.c_str(), rj.solve.iterations, rm.solve.iterations, rm.solve.multigrid_levels, dv,
                rj.after_stats->max_abs, rm.after_stats->max_abs);
  }
}

TEST(Mgpcg, EnclosedIsolatedAndMergedRegionsNeedNoFallback) {
  // The disconnected geometry has 3 enclosed Neumann regions (two cavity halves and an isolated
  // cell); the cavity halves merge on multigrid level 2. The projected preconditioner keeps CG in the
  // range of A, so no positivity failure or breakdown occurs.
  Grid3D mask = disconnected_mask();
  const auto map = classify_boundaries(mask);
  const PressureSystem sys(mask, map);
  Multigrid mg(mask, map, sys);
  ASSERT_GE(mg.levels(), 3u);
  EXPECT_EQ(mg.info(0).enclosed_components, 3u);
  EXPECT_EQ(mg.info(0).isolated_cells, 1u);
  EXPECT_LT(mg.info(2).components, mg.info(0).components);

  MacGrid mac(mask);
  initialize(mac, mask, 0.0, map);
  apply_velocity_bcs(mac, map, BoundaryConditions{kU});
  std::vector<double> b(sys.unknowns()), phi(sys.unknowns());
  sys.build_rhs(mac, b);
  // The cavity has a non-trivial (compatible) right-hand side.
  double cavity_rhs = 0.0;
  for (std::size_t u = 0; u < sys.unknowns(); ++u)
    if (!sys.components()[sys.component_of(u)].has_outlet) cavity_rhs += std::abs(b[u]);
  EXPECT_GT(cavity_rhs, 0.0);
  PcgWorkspace ws;
  SolveOptions options;
  options.preconditioner = &mg;
  options.preconditioner_levels = mg.levels();
  const auto st = solve_pressure(sys, b, phi, settings_for(PressureSolverKind::Mgpcg), ws, options);
  EXPECT_TRUE(st.converged);
  EXPECT_FALSE(st.fallback) << st.fallback_reason;
  EXPECT_EQ(st.used, PressureSolverKind::Mgpcg);
  EXPECT_EQ(st.setup_ms, 0.0);  // prebuilt hierarchy: nothing built here
  // The isolated cell has no equation: its potential is 0 (zero-mean gauge of a one-cell region).
  const auto pocket = static_cast<std::size_t>(sys.unknown_of_cell(mask.index(25, 1, 1)));
  EXPECT_EQ(phi[pocket], 0.0);
}

namespace {

class NegatingPreconditioner final : public PressurePreconditioner {
 public:
  void apply(std::span<const double> r, std::span<double> z) override {
    for (std::size_t n = 0; n < r.size(); ++n) z[n] = -r[n];
  }
};

// Behaves like the real V-cycle for `good` applications, then returns NaN.
class FailingLater final : public PressurePreconditioner {
 public:
  FailingLater(PressurePreconditioner& inner, int good) : inner_(inner), good_(good) {}
  void apply(std::span<const double> r, std::span<double> z) override {
    if (calls_++ < good_) {
      inner_.apply(r, z);
      return;
    }
    std::fill(z.begin(), z.end(), std::numeric_limits<double>::quiet_NaN());
  }

 private:
  PressurePreconditioner& inner_;
  int good_;
  int calls_ = 0;
};

}  // namespace

TEST(Mgpcg, FallsBackToJacobiWhenThePreconditionerIsNotPositive) {
  auto geo = std::move(geometries()[0]);
  const auto map = classify_boundaries(geo.mask);
  const PressureSystem sys(geo.mask, map);
  MacGrid mac(geo.mask);
  initialize(mac, geo.mask, geo.perturb, map);
  apply_velocity_bcs(mac, map, BoundaryConditions{kU});
  std::vector<double> b(sys.unknowns()), phi_ref(sys.unknowns()), phi(sys.unknowns());
  sys.build_rhs(mac, b);
  const auto ref = solve_pressure(sys, b, phi_ref, settings_for(PressureSolverKind::JacobiPcg));
  PcgWorkspace ws;

  // Negative definite from the start: r.z < 0 before the first iteration.
  NegatingPreconditioner neg;
  SolveOptions options;
  options.preconditioner = &neg;
  const auto st = solve_pressure(sys, b, phi, settings_for(PressureSolverKind::Mgpcg), ws, options);
  EXPECT_TRUE(st.fallback);
  EXPECT_NE(st.fallback_reason.find("lost positivity (r.z"), std::string::npos) << st.fallback_reason;
  EXPECT_EQ(st.fallback_iteration, 0u);
  EXPECT_EQ(st.requested, PressureSolverKind::Mgpcg);
  EXPECT_EQ(st.used, PressureSolverKind::JacobiPcg);
  EXPECT_TRUE(st.converged);
  EXPECT_EQ(st.iterations, ref.iterations);  // the full Jacobi solve from phi = 0
  EXPECT_LE(st.relative_residual, 1e-10);

  // Fails after a few good V-cycles: the Jacobi phase continues from the current iterate.
  Multigrid mg(geo.mask, map, sys);
  FailingLater later(mg, 4);
  options.preconditioner = &later;
  const auto st2 = solve_pressure(sys, b, phi, settings_for(PressureSolverKind::Mgpcg), ws, options);
  EXPECT_TRUE(st2.fallback);
  EXPECT_EQ(st2.fallback_iteration, 4u);  // V-cycles 1-4 were good (initial + 3); the 5th is NaN
  EXPECT_EQ(st2.used, PressureSolverKind::JacobiPcg);
  EXPECT_TRUE(st2.converged);
  EXPECT_LE(st2.relative_residual, 1e-10);
  std::printf("  late fallback after %zu MG iterations: %zu total iterations (Jacobi alone %zu); reason: %s\n",
              st2.fallback_iteration, st2.iterations, ref.iterations, st2.fallback_reason.c_str());
}

TEST(Mgpcg, SetupFailureIsReportedAndNotRetried) {
  auto geo = std::move(geometries()[0]);
  const auto map = classify_boundaries(geo.mask);
  auto settings = settings_for(PressureSolverKind::Mgpcg);
  settings.multigrid.post_sweeps = 1;  // asymmetric cycle: rejected by the multigrid setup
  Projector projector(geo.mask, map);
  for (int step = 0; step < 2; ++step) {
    MacGrid mac(geo.mask);
    initialize(mac, geo.mask, geo.perturb, map);
    const auto r = projector.project(mac, BoundaryConditions{kU}, settings, 1e-3);
    EXPECT_TRUE(r.solve.fallback);
    EXPECT_NE(r.solve.fallback_reason.find("multigrid setup failed"), std::string::npos) << r.solve.fallback_reason;
    EXPECT_NE(r.solve.fallback_reason.find("symmetric"), std::string::npos);
    EXPECT_EQ(r.solve.used, PressureSolverKind::JacobiPcg);
    EXPECT_TRUE(r.solve.converged);
    EXPECT_EQ(r.fluxes.wall_leak, 0.0);
  }
  EXPECT_EQ(projector.multigrid(), nullptr);
  EXPECT_EQ(projector.multigrid_setups(), 0u);
  EXPECT_FALSE(projector.multigrid_error().empty());

  // The one-shot path reports the same.
  MacGrid mac(geo.mask);
  initialize(mac, geo.mask, geo.perturb, map);
  const auto r = project_velocity(mac, geo.mask, map, BoundaryConditions{kU}, settings);
  EXPECT_TRUE(r.solve.fallback);
  EXPECT_TRUE(r.solve.converged);
}

TEST(Mgpcg, NonConvergenceIsStillAnError) {
  auto geo = std::move(geometries()[1]);
  const auto map = classify_boundaries(geo.mask);
  MacGrid mac(geo.mask);
  initialize(mac, geo.mask, geo.perturb, map);
  auto settings = settings_for(PressureSolverKind::Mgpcg, 1e-12);
  settings.max_iterations = 2;
  try {
    (void)project_velocity(mac, geo.mask, map, BoundaryConditions{kU}, settings);
    FAIL() << "expected projection_not_converged";
  } catch (const ProjectionError& e) {
    EXPECT_EQ(e.code(), "projection_not_converged");
  }
}

TEST(Mgpcg, WarmStartRescalingWorksIdenticallyWithMultigrid) {
  // Same construction as Projector.WarmStartCarriesThePressureAcrossTimestepChanges: b scales
  // exactly with dt, so the rescaled guess already solves the shortened and the following step.
  Grid3D mask(20, 10, 12, 2.0, 0.75, 1.2);
  mark_solid_box(mask, {{0.6, 0.15, 0.3}, {1.0, 0.6, 0.8}});
  const auto map = classify_boundaries(mask);
  auto field = [&](double dt) {
    MacGrid mac(mask);
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> d(-1.0, 1.0);
    auto fill = [&](StaggeredArray& a, const std::vector<BC>& roles) {
      for (std::size_t n = 0; n < a.size(); ++n) a.data()[n] = roles[n] == BC::Interior ? dt * d(rng) : 0.0;
    };
    fill(mac.u(), map.u);
    fill(mac.v(), map.v);
    fill(mac.w(), map.w);
    return mac;
  };
  for (const auto kind : {PressureSolverKind::JacobiPcg, PressureSolverKind::Mgpcg}) {
    Projector projector(mask, map);
    const auto settings = settings_for(kind);
    MacGrid a = field(2e-3), b = field(1e-4), c = field(2e-3);
    const auto r1 = projector.project(a, BoundaryConditions{0.0}, settings, 2e-3);
    const auto r2 = projector.project(b, BoundaryConditions{0.0}, settings, 1e-4);
    const auto r3 = projector.project(c, BoundaryConditions{0.0}, settings, 2e-3);
    SCOPED_TRACE(std::string(to_string(kind)));
    EXPECT_FALSE(r1.warm_started);
    EXPECT_GT(r1.solve.iterations, 0u);
    EXPECT_TRUE(r2.warm_started);
    EXPECT_EQ(r2.solve.iterations, 0u);
    EXPECT_TRUE(r3.warm_started);
    EXPECT_EQ(r3.solve.iterations, 0u);
    EXPECT_EQ(r2.solve.used, kind);
    for (std::size_t n = 0; n < a.p().data().size(); ++n) {
      EXPECT_NEAR(b.p().data()[n], a.p().data()[n], 1e-12 * (1 + std::abs(a.p().data()[n])));
    }
    if (kind == PressureSolverKind::Mgpcg) EXPECT_EQ(projector.multigrid_setups(), 1u);
  }
}

TEST(Mgpcg, RepeatedTimeStepsReuseTheHierarchyAndMatchJacobi) {
  Grid3D mask(32, 12, 12, 3.2, 1.08, 1.2);  // dx 0.1, dy 0.09, dz 0.1
  mark_solid_box(mask, {{0.8, 0.27, 0.3}, {1.2, 0.81, 0.9}});
  auto make = [&](PressureSolverKind kind) {
    SimulationSettings s;
    s.duration = 1.0;
    s.frame_interval = 1.0;
    s.tolerance = 1e-10;
    s.solver = kind;
    return std::make_unique<Simulation>(mask, s);
  };
  auto jac = make(PressureSolverKind::JacobiPcg);
  auto mg = make(PressureSolverKind::Mgpcg);
  const auto j0 = jac->initialize();
  const auto m0 = mg->initialize();
  EXPECT_GT(m0.setup_ms, 0.0);  // the hierarchy is built for the initial projection
  const Multigrid* hierarchy = mg->projector().multigrid();
  ASSERT_NE(hierarchy, nullptr);
  // Each run takes its own stable dt (they differ only at the solver-tolerance level); step 3 is a
  // shortened step of 0.05 dt.
  const double fractions[] = {1.0, 1.0, 0.05, 1.0, 1.0, 1.0};
  std::size_t it_j = 0, it_m = 0;
  for (const double f : fractions) {
    const auto sj = jac->advance(f * jac->proposed_dt());
    const auto sm = mg->advance(f * mg->proposed_dt());
    EXPECT_EQ(sm.solver, PressureSolverKind::Mgpcg);
    EXPECT_FALSE(sm.fallback);
    EXPECT_EQ(sm.setup_ms, 0.0);  // reused, never rebuilt
    EXPECT_EQ(sm.warm_started, sj.warm_started);
    EXPECT_LE(sm.relative_residual, 1e-10);
    EXPECT_EQ(sm.wall_leak, 0.0);
    EXPECT_NEAR(sm.outflow, sm.inflow, 1e-8 * sm.inflow);
    EXPECT_LE(sm.div_after_max, 10 * std::max(sj.div_after_max, 1e-12));
    it_j += sj.iterations;
    it_m += sm.iterations;
  }
  EXPECT_EQ(mg->projector().multigrid(), hierarchy);
  EXPECT_EQ(mg->projector().multigrid_setups(), 1u);
  EXPECT_EQ(jac->projector().multigrid_setups(), 0u);
  const double dv = max_abs_diff(velocities(mg->state()), velocities(jac->state()));
  EXPECT_LE(dv, 1e-6 * kU);
  std::printf("  6 steps (one at 0.05 dt): Jacobi %zu iterations, MGPCG %zu; max |du| after 6 steps %.2e m/s\n", it_j,
              it_m, dv);

  // run_simulation reports the solver diagnostics.
  SimulationSettings s;
  s.duration = 0.05;
  s.frame_interval = 0.05;
  s.solver = PressureSolverKind::Mgpcg;
  Simulation sim(mask, s);
  const auto summary = run_simulation(sim, [](std::size_t, const Simulation&) {}, {});
  EXPECT_EQ(summary.solver_requested, PressureSolverKind::Mgpcg);
  EXPECT_EQ(summary.multigrid_setups, 1u);
  EXPECT_GE(summary.multigrid_levels, 2u);
  EXPECT_GT(summary.multigrid_setup_ms, 0.0);
  EXPECT_GT(summary.multigrid_memory_bytes, 0u);
  EXPECT_EQ(summary.fallbacks, 0u);
  EXPECT_GT(summary.solve_seconds, 0.0);
}

TEST(Mgpcg, TrilinearTransfersProjectLikeJacobiPcg) {
  for (auto& geo : geometries()) {
    const auto map = classify_boundaries(geo.mask);
    MacGrid base(geo.mask);
    initialize(base, geo.mask, geo.perturb, map);
    MacGrid jac = base, mg = base;
    auto ms = settings_for(PressureSolverKind::Mgpcg);
    ms.multigrid.transfer = MultigridTransfer::Trilinear;
    const auto rj = project_velocity(jac, geo.mask, map, BoundaryConditions{kU}, settings_for(PressureSolverKind::JacobiPcg));
    const auto rm = project_velocity(mg, geo.mask, map, BoundaryConditions{kU}, ms);
    SCOPED_TRACE(geo.name);
    EXPECT_FALSE(rm.solve.fallback) << rm.solve.fallback_reason;
    EXPECT_EQ(rm.solve.used, PressureSolverKind::Mgpcg);
    EXPECT_LE(rm.solve.relative_residual, 1e-10);
    EXPECT_LE(max_abs_diff(velocities(mg), velocities(jac)), 1e-6 * kU);
    EXPECT_EQ(rm.fluxes.wall_leak, 0.0);
    EXPECT_NEAR(rm.fluxes.outlet, rm.fluxes.inlet, 1e-8 * rm.fluxes.inlet);
    std::printf("  %-36s trilinear transfers: MGPCG %3zu it (Jacobi %zu)\n", geo.name.c_str(), rm.solve.iterations,
                rj.solve.iterations);
  }
}

TEST(Mgpcg, OptionalMaxDivergenceStoppingTest) {
  auto geo = std::move(geometries()[1]);
  const auto map = classify_boundaries(geo.mask);
  for (const auto kind : {PressureSolverKind::JacobiPcg, PressureSolverKind::Mgpcg}) {
    SCOPED_TRACE(std::string(to_string(kind)));
    MacGrid a(geo.mask), b(geo.mask);
    initialize(a, geo.mask, geo.perturb, map);
    initialize(b, geo.mask, geo.perturb, map);
    const auto base = project_velocity(a, geo.mask, map, BoundaryConditions{kU}, settings_for(kind, 1e-6));
    // The reported max residual is the max divergence left after projection.
    EXPECT_NEAR(base.solve.max_residual, base.after_stats->max_abs, 1e-9 * base.solve.max_residual + 1e-15);
    auto strict = settings_for(kind, 1e-6);
    strict.max_divergence = 0.1 * base.solve.max_residual;
    const auto tight = project_velocity(b, geo.mask, map, BoundaryConditions{kU}, strict);
    EXPECT_TRUE(tight.solve.converged);
    EXPECT_GT(tight.solve.iterations, base.solve.iterations);
    EXPECT_LE(tight.after_stats->max_abs, *strict.max_divergence * (1 + 1e-9));
    EXPECT_LE(tight.solve.relative_residual, 1e-6);
  }
  ProjectionSettings bad;
  bad.max_divergence = 0.0;
  EXPECT_THROW(validate_projection_settings(bad), ProjectionError);
  // Unreachable bound: still an honest non-convergence error.
  MacGrid c(geo.mask);
  initialize(c, geo.mask, geo.perturb, map);
  auto impossible = settings_for(PressureSolverKind::Mgpcg);
  impossible.max_divergence = 1e-300;
  impossible.max_iterations = 50;
  EXPECT_THROW((void)project_velocity(c, geo.mask, map, BoundaryConditions{kU}, impossible), ProjectionError);
}

TEST(Mgpcg, ConfigSelectsThePressureSolver) {
  auto base = nlohmann::json::parse(R"({"domain": {"lx": 1, "ly": 1, "lz": 1}, "resolution": {"nx": 2, "ny": 2, "nz": 2}})");
  for (const char* block : {"projection", "simulation"}) {
    SCOPED_TRACE(block);
    auto j = base;
    j[block] = nlohmann::json::object();
    auto solver_of = [&](const GridConfig& c) {
      return std::string(block) == "projection" ? c.projection->pressure_solver : c.simulation->pressure_solver;
    };
    EXPECT_EQ(solver_of(parse_config(j)), PressureSolverKind::Auto);  // default
    for (const auto k : {PressureSolverKind::Auto, PressureSolverKind::Mgpcg, PressureSolverKind::JacobiPcg}) {
      j[block]["pressure_solver"] = std::string(to_string(k));
      EXPECT_EQ(solver_of(parse_config(j)), k);
    }
    for (const auto& bad : {nlohmann::json("multigrid"), nlohmann::json(1), nlohmann::json("MGPCG")}) {
      j[block]["pressure_solver"] = bad;
      try {
        (void)parse_config(j);
        FAIL() << bad.dump();
      } catch (const ConfigError& e) {
        EXPECT_NE(std::string(e.what()).find(std::string(block) + ".pressure_solver"), std::string::npos) << e.what();
      }
    }
  }
}
