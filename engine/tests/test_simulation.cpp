#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <vector>

#include "aeroflow/box.hpp"
#include "aeroflow/simulation.hpp"

using namespace aeroflow;
using BC = VelocityFaceBC;

namespace {

SimulationSettings fast(double duration = 0.05) {
  SimulationSettings s;
  s.duration = duration;
  s.frame_interval = duration;
  return s;
}

std::vector<double> velocities(const MacGrid& g) {
  std::vector<double> out;
  for (const StaggeredArray* a : {&g.u(), &g.v(), &g.w()}) out.insert(out.end(), a->data().begin(), a->data().end());
  return out;
}

double l2_difference(const MacGrid& a, const MacGrid& b) {
  const auto x = velocities(a), y = velocities(b);
  double s = 0.0;
  for (std::size_t n = 0; n < x.size(); ++n) s += (x[n] - y[n]) * (x[n] - y[n]);
  return std::sqrt(s / static_cast<double>(x.size()));
}

Grid3D box_tunnel(std::size_t nx = 32, std::size_t ny = 12, std::size_t nz = 12) {
  Grid3D mask(nx, ny, nz, 3.2, 1.08, 1.2);  // dx 0.1, dy 0.09, dz 0.1 (anisotropic)
  mark_solid_box(mask, {{0.8, 0.27, 0.3}, {1.2, 0.81, 0.9}});
  return mask;
}

}  // namespace

// --- 1/2. equilibrium and uniform flow ----------------------------------------------------------

TEST(Simulation, ZeroVelocityStaysAtRest) {
  const Grid3D mask(12, 6, 6, 1.2, 0.6, 0.6);
  auto s = fast(0.2);
  s.inlet_velocity = 0.0;
  s.initial = "rest";
  s.dt = 0.01;
  Simulation sim(mask, s);
  sim.initialize();
  for (int n = 0; n < 20; ++n) {
    const auto st = sim.advance(0.01);
    EXPECT_EQ(st.max_speed, 0.0);
    EXPECT_EQ(st.iterations, 0u);
  }
  for (double p : sim.state().p().data()) EXPECT_EQ(p, 0.0);
}

TEST(Simulation, UniformFlowInAnEmptyTunnelIsExactlySteady) {
  const Grid3D mask(20, 7, 9, 2.0, 0.63, 0.9);  // anisotropic
  Simulation sim(mask, fast(0.1));
  sim.initialize();
  const auto start = velocities(sim.state());
  std::size_t steps = 0;
  const auto summary = run_simulation(sim, [](std::size_t, const Simulation&) {},
                                      [&](const StepStats& st) {
                                        EXPECT_EQ(st.iterations, 0u);
                                        EXPECT_EQ(st.div_after_max, 0.0);
                                        ++steps;
                                      });
  EXPECT_EQ(velocities(sim.state()), start);  // bit for bit
  for (double p : sim.state().p().data()) EXPECT_EQ(p, 0.0);
  EXPECT_GT(summary.steps, 5u);
  EXPECT_EQ(summary.status, "completed");
}

// --- 4. viscous decay of a known mode ------------------------------------------------------------

TEST(Simulation, ViscousDecayMatchesTheDiscreteAndContinuousRates) {
  // Divergence-free (v, w) mode psi = A sin(pi y/Ly) sin(pi z/Lz), uniform in x, satisfying the
  // free-slip walls. Diffusion only (advection off: the test isolates nu lap u + projection).
  // Explicit diffusion spreads one cell per step, so inlet/outlet effects cannot reach the middle
  // of a long tunnel within the measured steps: there the amplitude must follow
  //   discrete:   (1 - nu dt lambda_h)^n,  lambda_h = 4/dy^2 sin^2(pi dy/2Ly) + 4/dz^2 sin^2(pi dz/2Lz)
  //   continuous: exp(-nu k^2 t),          k^2 = (pi/Ly)^2 + (pi/Lz)^2
  const std::size_t nx = 60, ny = 16, nz = 12;
  const double Ly = 1.0, Lz = 0.75, A = 1e-3, nu = 2e-3;
  const Grid3D mask(nx, ny, nz, 6.0, Ly, Lz);
  auto s = fast(1.0);
  s.inlet_velocity = 0.0;
  s.viscosity = nu;
  s.advection = false;
  s.tolerance = 1e-12;
  Simulation sim(mask, s);

  MacGrid init(mask);
  const double dy = init.dy(), dz = init.dz();
  auto psi = [&](double y, double z) { return A * std::sin(std::numbers::pi * y / Ly) * std::sin(std::numbers::pi * z / Lz); };
  for (std::size_t k = 0; k < nz; ++k)
    for (std::size_t j = 0; j <= ny; ++j)
      for (std::size_t i = 0; i < nx; ++i) {
        const double y = static_cast<double>(j) * dy;
        init.v()(i, j, k) = (psi(y, static_cast<double>(k + 1) * dz) - psi(y, static_cast<double>(k) * dz)) / dz;
      }
  for (std::size_t k = 0; k <= nz; ++k)
    for (std::size_t j = 0; j < ny; ++j)
      for (std::size_t i = 0; i < nx; ++i) {
        const double z = static_cast<double>(k) * dz;
        init.w()(i, j, k) = -(psi(static_cast<double>(j + 1) * dy, z) - psi(static_cast<double>(j) * dy, z)) / dy;
      }
  sim.initialize_from(init);
  const double v0 = sim.state().v()(nx / 2, ny / 2, nz / 2);
  ASSERT_GT(std::abs(v0), 1e-6);

  const double dt = 0.4 * sim.stability_limit();
  const int steps = 20;  // < nx / 2 cells: the middle is untouched by the x boundaries
  for (int n = 0; n < steps; ++n) sim.advance(dt);

  const double lambda_h = 4 / (dy * dy) * std::pow(std::sin(std::numbers::pi * dy / (2 * Ly)), 2) +
                          4 / (dz * dz) * std::pow(std::sin(std::numbers::pi * dz / (2 * Lz)), 2);
  const double k2 = std::pow(std::numbers::pi / Ly, 2) + std::pow(std::numbers::pi / Lz, 2);
  const double discrete = std::pow(1 - nu * dt * lambda_h, steps);
  const double continuous = std::exp(-nu * k2 * dt * steps);
  const double measured = sim.state().v()(nx / 2, ny / 2, nz / 2) / v0;
  std::printf("\n  decay over %d steps: measured %.10f  discrete %.10f  continuous %.10f\n", steps, measured, discrete,
              continuous);
  EXPECT_NEAR(measured, discrete, 1e-8);
  // Continuous agreement to O(h^2 + dt): the decay itself is ~16 %; the mismatch must be < 2 % of it.
  EXPECT_NEAR(measured, continuous, 0.02 * (1 - continuous));
  EXPECT_LT(measured, 1.0);
}

// --- 5/6/7. obstacles, fluxes, divergence -------------------------------------------------------

TEST(Simulation, ObstacleFlowConservesMassWithoutPenetration) {
  const auto mask = box_tunnel();
  auto s = fast(0.15);
  s.frame_interval = 0.05;
  Simulation sim(mask, s);
  std::size_t frames = 0;
  const auto summary = run_simulation(sim, [&](std::size_t, const Simulation&) { ++frames; },
                                      [&](const StepStats& st) {
                                        EXPECT_EQ(st.wall_leak, 0.0);
                                        EXPECT_NEAR(st.outflow, st.inflow, 1e-6 * st.inflow);
                                        EXPECT_LE(st.relative_residual, s.tolerance);
                                        if (st.div_before_rms > 0) {
                                          EXPECT_LE(st.div_after_rms, 10 * s.tolerance * st.div_before_rms);
                                        }
                                        EXPECT_LE(st.div_after_max, 1e-6 * 5.0 / 0.09);
                                      });
  EXPECT_EQ(summary.status, "completed");
  EXPECT_EQ(frames, 4u);  // t = 0, 0.05, 0.1, 0.15
  EXPECT_NEAR(summary.time, 0.15, 1e-12);
  // The inlet keeps its prescribed value exactly.
  const auto& map = sim.boundaries();
  for (std::size_t n = 0; n < map.u.size(); ++n)
    if (map.u[n] == BC::Inlet) EXPECT_EQ(sim.state().u().data()[n], 5.0);
  std::printf("\n  box tunnel 32x12x12: %zu steps, mean PCG iterations %.1f (initial projection %zu)\n", summary.steps,
              [&] {
                double sum = 0;
                for (std::size_t n = 1; n < summary.history.size(); ++n) sum += static_cast<double>(summary.history[n].iterations);
                return sum / static_cast<double>(summary.history.size() - 1);
              }(),
              summary.history[0].iterations);
}

TEST(Simulation, FirstStepIsColdAndShortenedStepsKeepTheWarmStart) {
  // The initial projection's impulse potential is not reused; later steps warm-start from the
  // previous pressure, rescaled to the new dt, including across a shortened (frame-landing) step.
  const auto mask = box_tunnel();
  Simulation sim(mask, fast(1.0));
  const StepStats init = sim.initialize();
  EXPECT_GT(init.iterations, 0u);
  EXPECT_FALSE(init.warm_started);

  std::vector<StepStats> steps;
  const double full = sim.proposed_dt();
  steps.push_back(sim.advance(sim.proposed_dt()));  // 1: cold
  steps.push_back(sim.advance(sim.proposed_dt()));  // 2: warm
  steps.push_back(sim.advance(0.05 * full));        // 3: shortened
  steps.push_back(sim.advance(sim.proposed_dt()));  // 4: the following full step
  EXPECT_FALSE(steps[0].warm_started);
  for (std::size_t n = 1; n < steps.size(); ++n) EXPECT_TRUE(steps[n].warm_started);
  for (const auto& st : steps) {
    EXPECT_LE(st.relative_residual, sim.settings().tolerance);
    EXPECT_EQ(st.wall_leak, 0.0);
    EXPECT_NEAR(st.outflow, st.inflow, 1e-6 * st.inflow);
  }
  // A warm start that is consistent with the pressure never costs more than the cold first step.
  EXPECT_LE(steps[1].iterations, steps[0].iterations);
  EXPECT_LE(steps[2].iterations, steps[0].iterations);
  EXPECT_LE(steps[3].iterations, steps[0].iterations);
  std::printf("\n  PCG iterations: initial projection %zu; steps cold %zu, warm %zu, short %zu, next full %zu\n",
              init.iterations, steps[0].iterations, steps[1].iterations, steps[2].iterations, steps[3].iterations);
}

// --- 9. timestep refinement ---------------------------------------------------------------------

TEST(Simulation, TimestepRefinementConvergesAtFirstOrder) {
  const auto mask = box_tunnel(24, 9, 9);
  auto run = [&](double cfl) {
    auto s = fast(0.08);
    s.cfl = cfl;
    auto sim = std::make_unique<Simulation>(mask, s);
    run_simulation(*sim, [](std::size_t, const Simulation&) {});
    return sim;
  };
  const auto a = run(0.4), b = run(0.2), c = run(0.1);
  const double e1 = l2_difference(a->state(), b->state());
  const double e2 = l2_difference(b->state(), c->state());
  std::printf("\n  dt refinement: |u(0.4)-u(0.2)| %.3e  |u(0.2)-u(0.1)| %.3e  ratio %.2f\n", e1, e2, e1 / e2);
  EXPECT_GT(e1, 0.0);
  EXPECT_NEAR(e1 / e2, 2.0, 0.4);
}

// --- energy ---------------------------------------------------------------------------------

TEST(Simulation, ViscosityDissipatesEnergyInAClosedCavity) {
  // Fluid cavity inside a solid shell: no inflow/outflow, no-slip walls. Kinetic energy must not grow.
  Grid3D mask(14, 14, 14, 1.4, 1.4, 1.4);
  mark_solid_box(mask, {{0.2, 0.2, 0.2}, {1.2, 1.2, 1.2}});
  for (std::size_t k = 4; k < 10; ++k)
    for (std::size_t j = 4; j < 10; ++j)
      for (std::size_t i = 4; i < 10; ++i) mask.set_cell(i, j, k, CellType::Fluid);
  auto s = fast(1.0);
  s.inlet_velocity = 0.0;
  s.viscosity = 5e-3;
  Simulation sim(mask, s);
  MacGrid init(mask);
  for (std::size_t k = 5; k < 10; ++k)  // swirl in the cavity: a rotation about z
    for (std::size_t j = 4; j < 10; ++j)
      for (std::size_t i = 5; i < 10; ++i) {
        init.u()(i, j, k) = (static_cast<double>(j) - 7.0) * 0.5;
        init.v()(i, j, k) = -(static_cast<double>(i) - 7.0) * 0.5;
      }
  sim.initialize_from(init);
  const double initial = sim.kinetic_energy();
  double previous = initial;
  ASSERT_GT(previous, 0.0);
  for (int n = 0; n < 60; ++n) {
    const auto st = sim.advance(sim.proposed_dt());
    EXPECT_LE(st.kinetic_energy, previous * (1 + 1e-12)) << "step " << n;
    previous = st.kinetic_energy;
  }
  EXPECT_LT(previous, initial);  // strictly dissipated overall
}

// --- 10. invalid settings and stability guards ---------------------------------------------------

TEST(Simulation, RejectsInvalidSettings) {
  const Grid3D mask(8, 4, 4, 0.8, 0.4, 0.4);
  auto expect_code = [&](SimulationSettings s, const char* code) {
    try {
      Simulation sim(mask, s);
      sim.initialize();
      FAIL() << "expected " << code;
    } catch (const SimulationError& e) {
      EXPECT_EQ(e.code(), code) << e.what();
    }
  };
  auto s = fast();
  s.viscosity = -1;
  expect_code(s, "invalid_settings");
  s = fast(); s.cfl = 0; expect_code(s, "invalid_settings");
  s = fast(); s.cfl = 1.5; expect_code(s, "invalid_settings");
  s = fast(); s.duration = 0; expect_code(s, "invalid_settings");
  s = fast(); s.frame_interval = -1; expect_code(s, "invalid_settings");
  s = fast(); s.max_steps = 0; expect_code(s, "invalid_settings");
  s = fast(); s.tolerance = 1; expect_code(s, "invalid_settings");
  s = fast(); s.initial = "random"; expect_code(s, "invalid_settings");
  s = fast(); s.dt = std::numeric_limits<double>::quiet_NaN(); expect_code(s, "invalid_settings");
  s = fast(); s.dt = 1.0; expect_code(s, "unstable_timestep");  // far above dx/U = 0.02
}

TEST(Simulation, UnstableStepsAndStepBudgetsAreErrors) {
  const Grid3D mask(8, 4, 4, 0.8, 0.4, 0.4);
  Simulation sim(mask, fast());
  sim.initialize();
  EXPECT_THROW(sim.advance(10 * sim.proposed_dt()), SimulationError);
  auto s = fast(10.0);
  s.max_steps = 5;
  Simulation small(mask, s);
  try {
    run_simulation(small, [](std::size_t, const Simulation&) {});
    FAIL() << "expected step_budget";
  } catch (const SimulationError& e) {
    EXPECT_EQ(e.code(), "step_budget");
  }
}

TEST(Simulation, UserTimestepIsAnUpperBoundAndFramesLandExactly) {
  const auto mask = box_tunnel(16, 6, 6);
  auto s = fast(0.1);
  s.frame_interval = 0.03;  // frames at 0, 0.03, 0.06, 0.09, 0.1
  s.dt = 0.004;
  Simulation sim(mask, s);
  std::vector<double> times;
  const auto summary = run_simulation(sim, [&](std::size_t, const Simulation& x) { times.push_back(x.time()); });
  ASSERT_EQ(times.size(), 5u);
  EXPECT_NEAR(times[1], 0.03, 1e-12);
  EXPECT_NEAR(times[3], 0.09, 1e-12);
  EXPECT_NEAR(times[4], 0.1, 1e-12);
  for (std::size_t n = 1; n < summary.history.size(); ++n) EXPECT_LE(summary.history[n].dt, 0.004 * (1 + 1e-12));
}

TEST(Simulation, WallTimeBudgetStopsCleanly) {
  const auto mask = box_tunnel(16, 6, 6);
  auto s = fast(0.5);
  s.max_wall_seconds = 1e-9;
  Simulation sim(mask, s);
  std::size_t frames = 0;
  const auto summary = run_simulation(sim, [&](std::size_t, const Simulation&) { ++frames; });
  EXPECT_EQ(summary.status, "stopped");
  EXPECT_EQ(summary.stopped_reason, "wall_time_budget");
  EXPECT_EQ(summary.steps, 1u);
  EXPECT_GE(frames, 1u);
}

// --- 11. multi-frame AFFIELD round trip ----------------------------------------------------------

#include <filesystem>

#include "aeroflow/field_export.hpp"
#include "aeroflow/field_io.hpp"

TEST(SimulationFields, MultiFrameAffieldRoundTrip) {
  const auto dir = std::filesystem::temp_directory_path() / "aeroflow_sim_frames";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const auto mask = box_tunnel(16, 6, 6);
  SimulationConfig cfg;
  cfg.duration = 0.06;
  cfg.frame_interval = 0.02;  // frames at 0, 0.02, 0.04, 0.06
  std::vector<nlohmann::json> progress;
  const auto out = export_fields(mask, {kSimPressure, kSimVelocityMagnitude, kSimDivergence, kSimU, kSimV, kSimW},
                                 dir.string(), {}, cfg, [&](const nlohmann::json& p) { progress.push_back(p); });
  ASSERT_FALSE(out.simulation.is_null());
  EXPECT_EQ(out.simulation["status"], "completed");
  EXPECT_EQ(out.simulation["frames"], 4);
  EXPECT_EQ(out.fields.size(), 6u * 4u);
  EXPECT_FALSE(progress.empty());
  EXPECT_EQ(progress.back()["time"].get<double>(), out.simulation["time"].get<double>());

  double previous_time = -1.0;
  for (int frame = 0; frame < 4; ++frame) {
    const auto p = read_field_file((dir / ("sim_pressure.f" + std::to_string(frame) + ".affield")).string());
    EXPECT_EQ(p.header["frame"]["index"], frame);
    const double t = p.header["frame"]["time_s"].get<double>();
    EXPECT_GT(t, previous_time);
    EXPECT_NEAR(t, 0.02 * frame, 1e-12);
    previous_time = t;
    EXPECT_EQ(p.header["provenance"]["kind"], "simulation");
    EXPECT_EQ(p.header["provenance"]["solved"], true);
    EXPECT_EQ(p.header["units"], "Pa");
    const auto u = read_field_file((dir / ("sim_u.f" + std::to_string(frame) + ".affield")).string());
    EXPECT_EQ(u.header["location"], "face_x");
    EXPECT_EQ(u.header["shape"], nlohmann::json::array({17, 6, 6}));
    const auto div = read_field_file((dir / ("sim_divergence.f" + std::to_string(frame) + ".affield")).string());
    EXPECT_LE(div.header["stats"]["max_abs"].get<double>(), 1e-5);
  }
  std::filesystem::remove_all(dir);
}

TEST(SimulationFields, ConfigBlockIsValidated) {
  auto base = nlohmann::json::parse(R"({"domain": {"lx": 1, "ly": 1, "lz": 1}, "resolution": {"nx": 2, "ny": 2, "nz": 2}})");
  auto with = [&](nlohmann::json sim) {
    auto j = base;
    j["simulation"] = std::move(sim);
    return j;
  };
  const auto ok = parse_config(with({{"duration", 0.2}, {"viscosity", 0.0}, {"initial", "rest"}, {"dt", nullptr}}));
  ASSERT_TRUE(ok.simulation.has_value());
  EXPECT_EQ(ok.simulation->duration, 0.2);
  EXPECT_EQ(ok.simulation->initial, "rest");
  for (const auto& bad : {nlohmann::json{{"cfl", 0}}, nlohmann::json{{"cfl", 1.5}}, nlohmann::json{{"viscosity", -1}},
                          nlohmann::json{{"duration", 0}}, nlohmann::json{{"frame_interval", 0}},
                          nlohmann::json{{"initial", "random"}}, nlohmann::json{{"max_steps", 0}},
                          nlohmann::json{{"duration", 100}, {"frame_interval", 0.001}}, nlohmann::json{{"nu", 1}}}) {
    EXPECT_THROW((void)parse_config(with(bad)), ConfigError) << bad.dump();
  }
}
