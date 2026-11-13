#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "aeroflow/boundary.hpp"
#include "aeroflow/grid3d.hpp"
#include "aeroflow/mac_grid.hpp"
#include "aeroflow/momentum.hpp"
#include "aeroflow/pressure.hpp"

namespace aeroflow {

// Time-dependent incompressible Navier-Stokes (Milestone 6): fractional-step projection,
// non-incremental (Chorin):
//   1. velocity BCs on u^n
//   2. u* = u^n + dt (-(u^n.grad)u^n + nu lap u^n)       on Interior faces (momentum.hpp)
//   3. prescribed BCs + zero-gradient outlet predictor on u*
//   4. A phi = -D u*  (Milestone 5 operator, PCG warm-started from (dt/rho) p^n; the first
//      step after initialization starts cold)
//   5. u^{n+1} = u* - G phi on Coupled/Outlet faces; prescribed BCs reapplied
//   6. p^{n+1} = rho phi / dt: a first-order (in dt) approximation of the physical gauge pressure,
//      p = 0 on the outlet plane. It is NOT an increment.
// dt is adaptive: dt = cfl * stability_limit(u^n, nu) (advective + diffusive), shortened to land on
// output frames and the end time, and capped by a user dt if one is given.

class SimulationError : public std::runtime_error {
 public:
  // code: invalid_settings, unstable_timestep, non_finite, step_budget
  SimulationError(std::string code, const std::string& message)
      : std::runtime_error(message), code_(std::move(code)) {}
  [[nodiscard]] const std::string& code() const noexcept { return code_; }

 private:
  std::string code_;
};

struct SimulationSettings {
  double inlet_velocity = 5.0;   // m/s, >= 0
  double density = 1.225;        // kg/m^3, > 0
  double viscosity = 1.5e-5;     // m^2/s, >= 0
  double duration = 0.5;         // s, > 0
  double frame_interval = 0.05;  // s, > 0
  double cfl = 0.5;              // in (0, 1]
  std::optional<double> dt;      // s; upper limit on every step, rejected if unstable at t = 0
  std::size_t max_steps = 2000;
  double tolerance = 1e-8;       // per-step PCG relative residual
  std::size_t max_iterations = 10000;
  std::string initial = "uniform";  // "uniform": U_in on active faces; "rest": 0 inside; both projected
  double max_wall_seconds = 900.0;  // hard stop (frames so far are kept)
  bool advection = true;            // switches for verification only
  bool diffusion = true;
  PressureSolverKind solver = PressureSolverKind::Auto;  // pressure solver (Milestone 7)
  MultigridSettings multigrid;      // used when solver == Mgpcg; the hierarchy is built once per run
  std::optional<double> max_divergence;  // optional extra max-norm stopping test (1/s), off by default
};

void validate_simulation_settings(const SimulationSettings& s);

struct StepStats {
  std::size_t step = 0;
  double time = 0.0;  // after the step
  double dt = 0.0;
  std::size_t iterations = 0;
  double relative_residual = 0.0;
  double div_before_max = 0.0, div_before_rms = 0.0;
  double div_after_max = 0.0, div_after_rms = 0.0;
  double inflow = 0.0, outflow = 0.0, wall_leak = 0.0;
  double max_speed = 0.0;  // max |face velocity| after the step
  double kinetic_energy = 0.0;
  bool dt_limited_by_stability = false;  // a user dt was reduced to the stable limit
  bool warm_started = false;             // pressure solve started from the previous step's pressure
  PressureSolverKind solver = PressureSolverKind::JacobiPcg;  // preconditioner that finished the solve
  bool fallback = false;                 // multigrid requested, Jacobi-PCG finished the solve
  double solve_ms = 0.0;                 // pressure PCG time
  double setup_ms = 0.0;                 // multigrid hierarchy setup charged to this step (first only)
};

class Simulation {
 public:
  // Validates settings and builds all state (no allocation happens inside step()).
  // `mask` must outlive the simulation.
  Simulation(const Grid3D& mask, SimulationSettings settings);

  // Sets the initial field (see SimulationSettings::initial), applies BCs and projects it so the
  // time loop starts divergence-free. Returns the projection statistics as step 0. The initial
  // projection's potential is not reused as a warm start (the first step solves cold).
  StepStats initialize();

  // Starts from a given velocity field instead (BCs applied, then projected). For verification
  // problems and restarts; the field must have the simulation's extents.
  StepStats initialize_from(const MacGrid& field);

  // Current explicit limit (before the CFL factor).
  [[nodiscard]] double stability_limit() const;
  // dt the next step would use (cfl * limit, capped by the user dt); +inf if unconstrained.
  [[nodiscard]] double proposed_dt() const;

  // Advances by `dt` (must not exceed cfl * stability_limit; throws unstable_timestep).
  StepStats advance(double dt);

  [[nodiscard]] const MacGrid& state() const noexcept { return mac_; }
  [[nodiscard]] const BoundaryMap& boundaries() const noexcept { return map_; }
  [[nodiscard]] const Grid3D& mask() const noexcept { return mask_; }
  [[nodiscard]] const DivergenceField& divergence() const noexcept { return projector_.divergence_after(); }
  [[nodiscard]] const SimulationSettings& settings() const noexcept { return settings_; }
  [[nodiscard]] double time() const noexcept { return time_; }
  [[nodiscard]] std::size_t steps() const noexcept { return steps_; }
  [[nodiscard]] double kinetic_energy() const;  // 1/2 rho sum(face velocity^2) * cell volume, J
  [[nodiscard]] std::size_t memory_bytes() const;
  [[nodiscard]] const Projector& projector() const noexcept { return projector_; }
  // First fallback reason seen in this run ("" if none).
  [[nodiscard]] const std::string& fallback_reason() const noexcept { return fallback_reason_; }

 private:
  StepStats record(const Projector::Result& r, double dt);

  const Grid3D& mask_;
  SimulationSettings settings_;
  BoundaryMap map_;
  MacGrid mac_;
  StaggeredArray next_u_, next_v_, next_w_;
  Projector projector_;
  BoundaryConditions bc_;
  ProjectionSettings projection_;
  MomentumTerms terms_;
  double time_ = 0.0;
  std::size_t steps_ = 0;
  std::string fallback_reason_;
};

struct RunSummary {
  std::size_t steps = 0;
  std::size_t frames = 0;
  double time = 0.0;
  std::string status = "completed";  // "completed" | "stopped"
  std::string stopped_reason;        // "wall_time_budget" when stopped
  std::size_t dt_reductions = 0;     // steps where a user dt was reduced for stability
  double wall_seconds = 0.0;
  std::vector<StepStats> history;    // step 0 = initial projection
  std::vector<double> frame_times;
  // Pressure solver diagnostics.
  PressureSolverKind solver_requested = PressureSolverKind::Auto;
  PressureSolverKind solver_selected = PressureSolverKind::JacobiPcg;  // Auto resolved for this grid
  std::size_t pressure_iterations = 0;  // all time steps (the initial projection excluded)
  std::size_t multigrid_levels = 0;
  std::size_t multigrid_setups = 0;    // 1 for an Mgpcg run (built once), 0 otherwise
  double multigrid_setup_ms = 0.0;
  std::size_t multigrid_memory_bytes = 0;
  std::size_t fallbacks = 0;           // projections finished by Jacobi-PCG after a multigrid failure
  std::string fallback_reason;         // the first one
  double solve_seconds = 0.0;          // total pressure PCG time
};

// Runs to settings.duration: frame 0 at t = 0, then one frame at each multiple of frame_interval
// (steps are shortened to land on them) and at the end time. `on_frame(index, simulation)` writes
// output; `on_step(stats)` reports progress. Throws SimulationError / ProjectionError on failure;
// stops cleanly (status "stopped") at max_wall_seconds.
RunSummary run_simulation(Simulation& sim, const std::function<void(std::size_t, const Simulation&)>& on_frame,
                          const std::function<void(const StepStats&)>& on_step = {});

}  // namespace aeroflow
