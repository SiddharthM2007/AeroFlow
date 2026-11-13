// Pressure-solver benchmark (Milestone 7). Not part of the engine or the test suite; built only
// with -DAEROFLOW_BUILD_BENCH=ON.
//
// For one grid and geometry it reports
//   * a single cold projection of the initialized field (Milestone 5 setting, tol 1e-10), and
//   * a sequence of Navier-Stokes steps (tol 1e-8) in which the SAME right-hand side is solved
//     three ways: cold (phi = 0), legacy warm start (previous phi as is, step 1 from the initial
//     impulse) and rescaled warm start (phi_prev * dt/dt_prev, step 1 cold; what Projector does).
//     The cold potential advances the flow, so all three columns see identical systems.
// One step (--short-step) uses 0.05 x the stable dt, like a frame-landing step.
//
// --multigrid instead reports the multigrid hierarchy (levels, memory, setup) and the cost of one
// V-cycle relative to one operator application. It does not solve anything with multigrid.
//
// --simulate jacobi|mgpcg runs the production Simulation (initial projection + --steps steps, one
// of them at 0.05 dt) with that pressure solver and reports wall time, iterations, setup cost,
// divergence, mass balance and peak memory. Run one process per solver for a clean peak RSS.
//
// Usage: pressure_bench --grid NX NY NZ [--domain LX LY LZ] [--geometry empty|rest|box|mesh]
//                       [--mesh FILE.aftri] [--steps N] [--short-step K] [--multigrid]
//                       [--simulate jacobi|mgpcg] [--sweeps N] [--transfer trilinear|constant]
//                       [--coarse-sweeps N] [--max-divergence D]

#include <sys/resource.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "aeroflow/box.hpp"
#include "aeroflow/mesh.hpp"
#include "aeroflow/momentum.hpp"
#include "aeroflow/multigrid.hpp"
#include "aeroflow/pressure.hpp"
#include "aeroflow/simulation.hpp"
#include "aeroflow/voxelize.hpp"

using namespace aeroflow;

namespace {

using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

double peak_rss_mb() {
  rusage r{};
  getrusage(RUSAGE_SELF, &r);
#ifdef __APPLE__
  return static_cast<double>(r.ru_maxrss) / 1048576.0;  // bytes
#else
  return static_cast<double>(r.ru_maxrss) / 1024.0;  // kilobytes
#endif
}

struct Options {
  std::size_t nx = 64, ny = 32, nz = 32;
  double lx = 4.0, ly = 2.0, lz = 2.0;
  std::string geometry = "box";  // empty: uniform U_in (b = 0); rest: interior at rest (impulsive start)
  std::string mesh;
  int steps = 12;
  int short_step = -1;  // default: steps / 2
  bool multigrid = false;
  std::string simulate;  // "", "jacobi" or "mgpcg"
  MultigridSettings mg;
  double max_divergence = 0.0;  // optional max-norm stopping test (0 = off)
};

Options parse(int argc, char** argv) {
  Options o;
  auto need = [&](int& n, int count) {
    if (n + count >= argc) throw std::invalid_argument(std::string(argv[n]) + " needs " + std::to_string(count) + " value(s)");
  };
  for (int n = 1; n < argc; ++n) {
    const std::string a = argv[n];
    if (a == "--grid") {
      need(n, 3);
      o.nx = std::stoul(argv[++n]);
      o.ny = std::stoul(argv[++n]);
      o.nz = std::stoul(argv[++n]);
    } else if (a == "--domain") {
      need(n, 3);
      o.lx = std::stod(argv[++n]);
      o.ly = std::stod(argv[++n]);
      o.lz = std::stod(argv[++n]);
    } else if (a == "--geometry") {
      need(n, 1);
      o.geometry = argv[++n];
    } else if (a == "--mesh") {
      need(n, 1);
      o.mesh = argv[++n];
    } else if (a == "--steps") {
      need(n, 1);
      o.steps = std::stoi(argv[++n]);
    } else if (a == "--simulate") {
      need(n, 1);
      o.simulate = argv[++n];
      if (o.simulate != "jacobi" && o.simulate != "mgpcg" && o.simulate != "compare") {
        throw std::invalid_argument("--simulate jacobi|mgpcg|compare");
      }
    } else if (a == "--sweeps") {
      need(n, 1);
      o.mg.pre_sweeps = o.mg.post_sweeps = std::stoul(argv[++n]);
    } else if (a == "--coarse-sweeps") {
      need(n, 1);
      o.mg.coarse_sweeps = std::stoul(argv[++n]);
    } else if (a == "--transfer") {
      need(n, 1);
      const std::string t = argv[++n];
      if (t != "trilinear" && t != "constant") throw std::invalid_argument("--transfer trilinear|constant");
      o.mg.transfer = t == "constant" ? MultigridTransfer::Constant : MultigridTransfer::Trilinear;
    } else if (a == "--max-divergence") {
      need(n, 1);
      o.max_divergence = std::stod(argv[++n]);
    } else if (a == "--multigrid") {
      o.multigrid = true;
    } else if (a == "--short-step") {
      need(n, 1);
      o.short_step = std::stoi(argv[++n]);
    } else {
      throw std::invalid_argument("unknown argument " + a);
    }
  }
  if (o.geometry != "empty" && o.geometry != "rest" && o.geometry != "box" && o.geometry != "mesh") {
    throw std::invalid_argument("--geometry must be empty, rest, box or mesh");
  }
  if (o.geometry == "mesh" && o.mesh.empty()) throw std::invalid_argument("--geometry mesh needs --mesh FILE.aftri");
  if (o.short_step < 0) o.short_step = o.steps / 2;
  return o;
}

int run(const Options& o) {
  constexpr double kInlet = 5.0, kNu = 1.5e-5, kCfl = 0.5;
  Grid3D mask(o.nx, o.ny, o.nz, o.lx, o.ly, o.lz);
  auto t0 = Clock::now();
  if (o.geometry == "box") {
    // The viewer's default procedural box, scaled with the domain.
    mark_solid_box(mask, {{0.25 * o.lx, 0.0, 0.375 * o.lz}, {0.375 * o.lx, 0.25 * o.ly, 0.625 * o.lz}});
  } else if (o.geometry == "mesh") {
    voxelize_mesh(mask, read_triangle_file(o.mesh));
  }
  const double geometry_ms = ms_since(t0);
  std::size_t solid = 0;
  for (const auto c : mask.cells()) solid += c == CellType::Solid ? 1 : 0;

  if (!o.simulate.empty()) {
    auto run_one = [&](PressureSolverKind kind) {
    SimulationSettings ss;
    ss.inlet_velocity = kInlet;
    ss.viscosity = kNu;
    ss.cfl = kCfl;
    ss.duration = 1e9;
    ss.frame_interval = 1e9;
    ss.max_steps = 1000000;
    ss.initial = o.geometry == "rest" ? "rest" : "uniform";
    ss.solver = kind;
    ss.multigrid = o.mg;
    if (o.max_divergence > 0) ss.max_divergence = o.max_divergence;
    const auto wall = Clock::now();
    auto owned = std::make_unique<Simulation>(mask, ss);
    Simulation& sim = *owned;
    const auto init = sim.initialize();
    const double init_ms = ms_since(wall);
    double solve_ms = 0, step_ms = 0, worst_div = 0, worst_flux = 0, worst_leak = 0, worst_res = 0;
    std::size_t iterations = 0, max_it = 0, fallbacks = 0;
    for (int k = 1; k <= o.steps; ++k) {
      const double dt = (k == o.short_step ? 0.05 : 1.0) * sim.proposed_dt();
      const auto t = Clock::now();
      const auto st = sim.advance(dt);
      step_ms += ms_since(t);
      solve_ms += st.solve_ms;
      iterations += st.iterations;
      max_it = std::max(max_it, st.iterations);
      fallbacks += st.fallback ? 1 : 0;
      worst_div = std::max(worst_div, st.div_after_max);
      worst_res = std::max(worst_res, st.relative_residual);
      worst_flux = std::max(worst_flux, st.inflow > 0 ? std::abs(st.outflow / st.inflow - 1.0) : 0.0);
      worst_leak = std::max(worst_leak, st.wall_leak);
    }
    const double total_ms = ms_since(wall);
    const auto* mg = sim.projector().multigrid();
    // Where is the largest remaining divergence (= residual) after the last step?
    {
      const auto& div = sim.divergence();
      std::size_t at = 0;
      double best = 0.0, sumsq = 0.0;
      std::size_t count = 0;
      for (std::size_t c = 0; c < mask.cell_count(); ++c) {
        if (mask.cells()[c] != CellType::Fluid) continue;
        const double v = std::abs(div.values.data()[c]);
        sumsq += v * v;
        ++count;
        if (v > best) {
          best = v;
          at = c;
        }
      }
      const std::size_t i = at % o.nx, j = (at / o.nx) % o.ny, k = at / (o.nx * o.ny);
      bool near_solid = false;
      const long d[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
      for (const auto& q : d) {
        const long a = static_cast<long>(i) + q[0], b = static_cast<long>(j) + q[1], e = static_cast<long>(k) + q[2];
        if (a < 0 || b < 0 || e < 0 || a >= static_cast<long>(o.nx) || b >= static_cast<long>(o.ny) || e >= static_cast<long>(o.nz)) continue;
        near_solid = near_solid || mask.cell(static_cast<std::size_t>(a), static_cast<std::size_t>(b), static_cast<std::size_t>(e)) == CellType::Solid;
      }
      const bool on_wall = i == 0 || j == 0 || k == 0 || i + 1 == o.nx || j + 1 == o.ny || k + 1 == o.nz;
      std::printf("  final max |div| %.2e at cell (%zu,%zu,%zu)%s%s; rms %.2e (max/rms %.0f)\n", best, i, j, k,
                  near_solid ? " next to the obstacle" : "", on_wall ? " on a domain boundary" : "",
                  std::sqrt(sumsq / static_cast<double>(count)), best / std::sqrt(sumsq / static_cast<double>(count)));
    }
    const double s = o.steps;
    std::printf("SIMULATE grid=%zux%zux%zu geometry=%s solver=%s fluid=%zu steps=%d\n", o.nx, o.ny, o.nz,
                o.geometry.c_str(), std::string(to_string(kind)).c_str(), sim.projector().system().unknowns(), o.steps);
    std::printf("  total_wall_s=%.2f init_ms=%.0f (initial projection %zu it, multigrid setup %.1f ms, levels %zu)\n",
                total_ms / 1000.0, init_ms, init.iterations, init.setup_ms, mg ? mg->levels() : 0);
    std::printf("  per_step_ms=%.1f pressure_solve_ms=%.1f other_ms=%.1f mean_it=%.1f max_it=%zu fallbacks=%zu\n",
                step_ms / s, solve_ms / s, (step_ms - solve_ms) / s, static_cast<double>(iterations) / s, max_it, fallbacks);
    std::printf("  worst rel_residual=%.2e div_max=%.2e |q_out/q_in-1|=%.1e wall_leak=%.1e\n", worst_res, worst_div,
                worst_flux, worst_leak);
    std::printf("  memory: peak_rss_mb=%.0f simulation_mb=%.1f multigrid_mb=%.1f\n", peak_rss_mb(),
                static_cast<double>(sim.memory_bytes()) / 1048576.0,
                mg ? static_cast<double>(mg->memory_bytes()) / 1048576.0 : 0.0);
    return owned;
    };
    if (o.simulate != "compare") {
      (void)run_one(o.simulate == "mgpcg" ? PressureSolverKind::Mgpcg : PressureSolverKind::JacobiPcg);
      return 0;
    }
    // Same grid, geometry, settings and step sequence with both solvers; compare the final states.
    const auto jac = run_one(PressureSolverKind::JacobiPcg);
    const auto mg = run_one(PressureSolverKind::Mgpcg);
    double du = 0.0, umax = 0.0, dp = 0.0, pmax = 0.0;
    const StaggeredArray* fj[3] = {&jac->state().u(), &jac->state().v(), &jac->state().w()};
    const StaggeredArray* fm[3] = {&mg->state().u(), &mg->state().v(), &mg->state().w()};
    for (int a = 0; a < 3; ++a)
      for (std::size_t n = 0; n < fj[a]->size(); ++n) {
        du = std::max(du, std::abs(fj[a]->data()[n] - fm[a]->data()[n]));
        umax = std::max(umax, std::abs(fj[a]->data()[n]));
      }
    for (std::size_t n = 0; n < jac->state().p().size(); ++n) {
      dp = std::max(dp, std::abs(jac->state().p().data()[n] - mg->state().p().data()[n]));
      pmax = std::max(pmax, std::abs(jac->state().p().data()[n]));
    }
    std::printf("COMPARE final time jacobi %.6g s / mgpcg %.6g s; max |du| = %.2e m/s (%.1e of max |u|), "
                "max |dp| = %.2e Pa (%.1e of max |p|)\n",
                jac->time(), mg->time(), du, du / umax, dp, dp / pmax);
    return 0;
  }

  const auto map = classify_boundaries(mask);
  const BoundaryConditions bc{kInlet};
  MacGrid mac(mask);
  if (o.geometry != "rest") {
    const FaceTypes faces = classify_faces(mask);
    mac.set_uniform_velocity({kInlet, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  }
  apply_velocity_bcs(mac, map, bc);

  t0 = Clock::now();
  const PressureSystem sys(mask, map);
  const double setup_ms = ms_since(t0);
  const std::size_t n = sys.unknowns();
  std::vector<double> b(n), phi(n), legacy(n), rescaled(n), guess(n);
  PcgWorkspace ws;
  DivergenceField div;

  if (o.multigrid) {
    const double rss_before = peak_rss_mb();
    Multigrid mg(mask, map, sys, o.mg);
    std::printf("CASE grid=%zux%zux%zu geometry=%s fluid=%zu pressure_system_mb=%.1f\n", o.nx, o.ny, o.nz,
                o.geometry.c_str(), n, static_cast<double>(sys.memory_bytes()) / 1048576.0);
    std::printf("MG setup_ms=%.1f levels=%zu memory_mb=%.1f (%.1f bytes per fine unknown)\n", mg.setup_ms(), mg.levels(),
                static_cast<double>(mg.memory_bytes()) / 1048576.0,
                static_cast<double>(mg.memory_bytes()) / static_cast<double>(n));
    for (std::size_t l = 0; l < mg.levels(); ++l) {
      const auto& i = mg.info(l);
      std::printf("  level %zu: %4zux%4zux%4zu coarsened=%d%d%d unknowns=%9zu components=%zu enclosed=%zu "
                  "isolated=%zu memory_mb=%.2f\n",
                  l, i.dims[0], i.dims[1], i.dims[2], i.coarsened[0], i.coarsened[1], i.coarsened[2], i.unknowns,
                  i.components, i.enclosed_components, i.isolated_cells, static_cast<double>(i.memory_bytes) / 1048576.0);
    }
    sys.build_rhs(mac, b);
    std::vector<double> z(n), y(n);
    const int reps = 5;
    t0 = Clock::now();
    for (int r = 0; r < reps; ++r) sys.apply(b, y);
    const double apply_ms = ms_since(t0) / reps;
    mg.apply(b, z);  // warm caches
    t0 = Clock::now();
    for (int r = 0; r < reps; ++r) mg.apply(b, z);
    const double vcycle_ms = ms_since(t0) / reps;
    mg.set_profiling(true);
    for (int r = 0; r < reps; ++r) mg.apply(b, z);
    const auto& pr = mg.profile();
    const double c = static_cast<double>(pr.cycles);
    std::printf("PROFILE per V-cycle ms: fine smooth %.2f, fine residual %.2f, fine restrict %.2f, fine prolong %.2f, "
                "coarse levels %.2f\n",
                pr.fine_smooth / c, pr.fine_residual / c, pr.fine_restrict / c, pr.fine_prolong / c, pr.coarse_levels / c);
    double rz = 0.0;
    for (std::size_t u = 0; u < n; ++u) rz += b[u] * z[u];
    std::printf("COST operator_apply_ms=%.2f vcycle_ms=%.2f vcycle/apply=%.1f r.Br=%.3e finite=%d\n", apply_ms,
                vcycle_ms, vcycle_ms / apply_ms, rz, std::isfinite(rz) ? 1 : 0);
    std::printf("MEMORY peak_rss_mb=%.0f (before hierarchy %.0f)\n", peak_rss_mb(), rss_before);
    return 0;
  }

  ProjectionSettings ps;
  ps.tolerance = 1e-10;
  sys.build_rhs(mac, b);
  t0 = Clock::now();
  auto st = solve_pressure(sys, b, phi, ps, ws, {false, false});
  const double solve_ms = ms_since(t0);
  sys.correct_velocity(mac, phi);
  apply_prescribed_velocity_bcs(mac, map, bc);
  compute_divergence_into(mac, mask, div);
  auto ds = divergence_stats(div, mask);
  auto fl = boundary_fluxes(mac, map);
  std::printf("CASE grid=%zux%zux%zu domain=%gx%gx%g geometry=%s cells=%zu fluid=%zu solid=%zu components=%zu "
              "geometry_ms=%.0f setup_ms=%.1f\n",
              o.nx, o.ny, o.nz, o.lx, o.ly, o.lz, o.geometry.c_str(), mask.cell_count(), n, solid,
              sys.components().size(), geometry_ms, setup_ms);
  std::printf("PROJECTION it=%zu solve_ms=%.0f rel_residual=%.2e div_max=%.2e div_rms=%.2e q_in=%.6f q_out=%.6f "
              "wall_leak=%.1e\n",
              st.iterations, solve_ms, st.relative_residual, ds ? ds->max_abs : 0.0, ds ? ds->rms : 0.0, fl.inlet,
              fl.outlet, fl.wall_leak);

  ps.tolerance = 1e-8;
  StaggeredArray ou(o.nx + 1, o.ny, o.nz), ov(o.nx, o.ny + 1, o.nz), ow(o.nx, o.ny, o.nz + 1);
  const MomentumTerms terms{true, true, kNu};
  legacy = phi;  // the legacy warm start reused the initial impulse potential
  double prev_dt = 0.0;
  double t_mom = 0, t_cold = 0, t_legacy = 0, t_rescaled = 0, t_rest = 0;
  std::size_t it_cold = 0, it_legacy = 0, it_rescaled = 0;
  double worst_div = 0, worst_flux = 0, worst_leak = 0;
  std::printf("STEP  dt/limit  it_cold it_legacy it_rescaled   ms_cold ms_legacy ms_rescaled    div_max  q_out/q_in-1  wall_leak\n");
  for (int k = 1; k <= o.steps; ++k) {
    const double limit = kCfl * stability_limit(mac, kNu);
    const double dt = k == o.short_step ? 0.05 * limit : limit;
    t0 = Clock::now();
    apply_velocity_bcs(mac, map, bc);
    explicit_momentum_update(mac, map, terms, dt, ou, ov, ow);
    std::swap(mac.u(), ou);
    std::swap(mac.v(), ov);
    std::swap(mac.w(), ow);
    apply_velocity_bcs(mac, map, bc);
    sys.build_rhs(mac, b);
    t_mom += ms_since(t0);

    t0 = Clock::now();
    const auto c = solve_pressure(sys, b, phi, ps, ws, {false, false});
    const double mc = ms_since(t0);
    guess = legacy;
    t0 = Clock::now();
    const auto l = solve_pressure(sys, b, guess, ps, ws, {true, false});
    const double ml = ms_since(t0);
    legacy = guess;
    const bool warm = k > 1;
    if (warm) {
      for (std::size_t u = 0; u < n; ++u) guess[u] = rescaled[u] * (dt / prev_dt);
    }
    t0 = Clock::now();
    const auto r = solve_pressure(sys, b, guess, ps, ws, {warm, false});
    const double mr = ms_since(t0);
    rescaled = guess;
    prev_dt = dt;

    t0 = Clock::now();
    sys.correct_velocity(mac, phi);
    apply_prescribed_velocity_bcs(mac, map, bc);
    compute_divergence_into(mac, mask, div);
    ds = divergence_stats(div, mask);
    fl = boundary_fluxes(mac, map);
    t_rest += ms_since(t0);
    const double div_max = ds ? ds->max_abs : 0.0;
    const double flux = fl.inlet > 0 ? fl.outlet / fl.inlet - 1.0 : 0.0;
    worst_div = std::max(worst_div, div_max);
    worst_flux = std::max(worst_flux, std::abs(flux));
    worst_leak = std::max(worst_leak, fl.wall_leak);
    std::printf("%4d  %8.3f  %7zu %9zu %11zu  %8.0f %9.0f %11.0f  %9.2e  %12.1e  %9.1e\n", k, dt / limit, c.iterations,
                l.iterations, r.iterations, mc, ml, mr, div_max, flux, fl.wall_leak);
    std::fflush(stdout);
    t_cold += mc;
    t_legacy += ml;
    t_rescaled += mr;
    it_cold += c.iterations;
    it_legacy += l.iterations;
    it_rescaled += r.iterations;
  }
  const double s = o.steps;
  std::printf("MEAN it cold/legacy/rescaled = %.1f / %.1f / %.1f ; solve_ms = %.0f / %.0f / %.0f ; momentum+rhs_ms=%.0f "
              "correction+diagnostics_ms=%.0f\n",
              it_cold / s, it_legacy / s, it_rescaled / s, t_cold / s, t_legacy / s, t_rescaled / s, t_mom / s, t_rest / s);
  std::printf("WORST div_max=%.2e |q_out/q_in-1|=%.1e wall_leak=%.1e\n", worst_div, worst_flux, worst_leak);
  std::printf("MEMORY peak_rss_mb=%.0f pressure_system_mb=%.1f\n", peak_rss_mb(),
              static_cast<double>(sys.memory_bytes()) / 1048576.0);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(parse(argc, argv));
  } catch (const std::exception& e) {
    std::fprintf(stderr, "pressure_bench: %s\n", e.what());
    return 2;
  }
}
