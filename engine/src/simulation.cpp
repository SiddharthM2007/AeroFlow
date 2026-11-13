#include "aeroflow/simulation.hpp"

#include "aeroflow/multigrid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

namespace aeroflow {

namespace {

std::string fmt(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.4g", v);
  return buf;
}

void require(bool ok, const std::string& message) {
  if (!ok) throw SimulationError("invalid_settings", message);
}

bool finite_positive(double v) { return std::isfinite(v) && v > 0.0; }

// Relative slack for landing exactly on frame/end times despite rounding.
constexpr double kTimeEps = 1e-12;

}  // namespace

void validate_simulation_settings(const SimulationSettings& s) {
  require(std::isfinite(s.inlet_velocity) && s.inlet_velocity >= 0.0, "inlet_velocity must be finite and >= 0 m/s");
  require(finite_positive(s.density), "density must be a finite number > 0 kg/m^3");
  require(std::isfinite(s.viscosity) && s.viscosity >= 0.0, "viscosity must be finite and >= 0 m^2/s");
  require(finite_positive(s.duration), "duration must be a finite number > 0 s");
  require(finite_positive(s.frame_interval), "frame_interval must be a finite number > 0 s");
  require(std::isfinite(s.cfl) && s.cfl > 0.0 && s.cfl <= 1.0, "cfl must lie in (0, 1]");
  require(!s.dt || finite_positive(*s.dt), "dt must be a finite number > 0 s");
  require(s.max_steps >= 1, "max_steps must be at least 1");
  require(std::isfinite(s.tolerance) && s.tolerance > 0.0 && s.tolerance < 1.0, "tolerance must lie in (0, 1)");
  require(s.max_iterations >= 1, "max_iterations must be at least 1");
  require(s.initial == "uniform" || s.initial == "rest", "initial must be \"uniform\" or \"rest\"");
  require(finite_positive(s.max_wall_seconds), "max_wall_seconds must be a finite number > 0");
  require(s.duration / s.frame_interval <= 10000.0, "duration / frame_interval must not exceed 10000 frames");
}

Simulation::Simulation(const Grid3D& mask, SimulationSettings settings)
    : mask_(mask),
      settings_(std::move(settings)),
      map_(classify_boundaries(mask)),
      mac_(mask),
      next_u_(mac_.u().n0(), mac_.u().n1(), mac_.u().n2()),
      next_v_(mac_.v().n0(), mac_.v().n1(), mac_.v().n2()),
      next_w_(mac_.w().n0(), mac_.w().n1(), mac_.w().n2()),
      projector_(mask_, map_) {
  validate_simulation_settings(settings_);
  bc_.inlet_velocity = settings_.inlet_velocity;
  validate_boundaries(map_, bc_);
  projection_.density = settings_.density;
  projection_.tolerance = settings_.tolerance;
  projection_.max_iterations = settings_.max_iterations;
  projection_.solver = settings_.solver;
  projection_.multigrid = settings_.multigrid;
  projection_.max_divergence = settings_.max_divergence;
  terms_.advection = settings_.advection;
  terms_.diffusion = settings_.diffusion;
  terms_.viscosity = settings_.viscosity;
}

double Simulation::kinetic_energy() const {
  double sum = 0.0;
  for (const StaggeredArray* f : {&mac_.u(), &mac_.v(), &mac_.w()})
    for (double v : f->data()) sum += v * v;
  return 0.5 * settings_.density * sum * mac_.dx() * mac_.dy() * mac_.dz();
}

std::size_t Simulation::memory_bytes() const {
  return MacGrid::required_bytes(mac_.nx(), mac_.ny(), mac_.nz()) +
         (next_u_.size() + next_v_.size() + next_w_.size()) * sizeof(double) + projector_.memory_bytes() +
         map_.u.size() + map_.v.size() + map_.w.size() + mask_.cell_count();
}

double Simulation::stability_limit() const { return aeroflow::stability_limit(mac_, settings_.viscosity); }

double Simulation::proposed_dt() const {
  double dt = settings_.cfl * stability_limit();
  if (settings_.dt) dt = std::min(dt, *settings_.dt);
  return dt;
}

StepStats Simulation::record(const Projector::Result& r, double dt) {
  StepStats s;
  s.step = steps_;
  s.time = time_;
  s.dt = dt;
  s.iterations = r.solve.iterations;
  s.warm_started = r.warm_started;
  s.solver = r.solve.used;
  s.fallback = r.solve.fallback;
  s.solve_ms = r.solve.solve_ms;
  s.setup_ms = r.solve.setup_ms;
  if (r.solve.fallback && fallback_reason_.empty()) fallback_reason_ = r.solve.fallback_reason;
  s.relative_residual = r.solve.relative_residual;
  if (r.before) {
    s.div_before_max = r.before->max_abs;
    s.div_before_rms = r.before->rms;
  }
  if (r.after) {
    s.div_after_max = r.after->max_abs;
    s.div_after_rms = r.after->rms;
  }
  s.inflow = r.fluxes.inlet;
  s.outflow = r.fluxes.outlet;
  s.wall_leak = r.fluxes.wall_leak;
  const VelocityBounds b = velocity_bounds(mac_);
  if (!b.finite) throw SimulationError("non_finite", "velocity became non-finite at step " + std::to_string(steps_));
  s.max_speed = std::max({b.max_u, b.max_v, b.max_w});
  s.kinetic_energy = kinetic_energy();
  return s;
}

StepStats Simulation::initialize() {
  mac_.set_zero();
  if (settings_.initial == "uniform") {
    const FaceTypes faces = classify_faces(mask_);
    mac_.set_uniform_velocity({settings_.inlet_velocity, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  }
  // Pressure scale for the initial projection: the first step's dt (or the user dt).
  apply_velocity_bcs(mac_, map_, bc_);
  double dt0 = proposed_dt();
  if (!std::isfinite(dt0)) dt0 = settings_.frame_interval;
  const auto r = projector_.project(mac_, bc_, projection_, dt0);
  // The initial potential is an impulse that removes the divergence of a prescribed field, not a
  // pressure of the flow: the first physical step starts cold.
  projector_.reset_warm_start();
  time_ = 0.0;
  steps_ = 0;
  // A user dt is checked against the projected (divergence-free) field, whose peak speed
  // around obstacles exceeds U_in.
  if (settings_.dt) {
    const double limit = settings_.cfl * stability_limit();
    if (*settings_.dt > limit) {
      throw SimulationError("unstable_timestep",
                            "dt " + fmt(*settings_.dt) + " s exceeds the stable explicit limit " + fmt(limit) +
                                " s at t = 0 (cfl " + fmt(settings_.cfl) + " x 1/(sum |u|/h + 2 nu sum 1/h^2))");
    }
  }
  return record(r, 0.0);
}

StepStats Simulation::initialize_from(const MacGrid& field) {
  if (field.nx() != mac_.nx() || field.ny() != mac_.ny() || field.nz() != mac_.nz()) {
    throw SimulationError("invalid_settings", "initial field does not match the simulation grid");
  }
  mac_.u() = field.u();
  mac_.v() = field.v();
  mac_.w() = field.w();
  apply_velocity_bcs(mac_, map_, bc_);
  double dt0 = proposed_dt();
  if (!std::isfinite(dt0)) dt0 = settings_.frame_interval;
  const auto r = projector_.project(mac_, bc_, projection_, dt0);
  // The initial potential is an impulse that removes the divergence of a prescribed field, not a
  // pressure of the flow: the first physical step starts cold.
  projector_.reset_warm_start();
  time_ = 0.0;
  steps_ = 0;
  return record(r, 0.0);
}

StepStats Simulation::advance(double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt)) throw SimulationError("invalid_settings", "dt must be a finite number > 0 s");
  const double limit = settings_.cfl * stability_limit();
  if (dt > limit * (1.0 + 1e-12)) {
    throw SimulationError("unstable_timestep", "dt " + fmt(dt) + " s exceeds the stable explicit limit " + fmt(limit) +
                                                   " s at t = " + fmt(time_) + " s");
  }
  apply_velocity_bcs(mac_, map_, bc_);
  explicit_momentum_update(mac_, map_, terms_, dt, next_u_, next_v_, next_w_);
  std::swap(mac_.u(), next_u_);
  std::swap(mac_.v(), next_v_);
  std::swap(mac_.w(), next_w_);
  const VelocityBounds b = velocity_bounds(mac_);
  if (!b.finite) {
    throw SimulationError("non_finite", "intermediate velocity became non-finite at step " + std::to_string(steps_ + 1));
  }
  const auto r = projector_.project(mac_, bc_, projection_, dt);
  time_ += dt;
  ++steps_;
  return record(r, dt);
}

RunSummary run_simulation(Simulation& sim, const std::function<void(std::size_t, const Simulation&)>& on_frame,
                          const std::function<void(const StepStats&)>& on_step) {
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  const SimulationSettings& s = sim.settings();
  RunSummary out;
  out.history.reserve(std::min<std::size_t>(s.max_steps + 1, 100000));

  const StepStats init = sim.initialize();
  out.history.push_back(init);
  if (on_step) on_step(init);

  // Preflight on the projected initial field: the run must fit in max_steps at the current dt.
  const double dt0 = sim.proposed_dt();
  if (std::isfinite(dt0)) {
    const double estimate = std::ceil(s.duration / dt0);
    if (estimate > static_cast<double>(s.max_steps)) {
      throw SimulationError("step_budget", "duration " + fmt(s.duration) + " s needs about " + fmt(estimate) +
                                               " steps at the stable dt " + fmt(dt0) + " s, more than max_steps " +
                                               std::to_string(s.max_steps));
    }
  }

  on_frame(0, sim);
  out.frames = 1;
  out.frame_times.push_back(0.0);
  std::size_t next_frame = 1;
  const double end = s.duration;

  while (sim.time() < end * (1.0 - kTimeEps)) {
    if (sim.steps() >= s.max_steps) {
      throw SimulationError("step_budget", "reached max_steps " + std::to_string(s.max_steps) + " at t = " +
                                               fmt(sim.time()) + " s before the end time " + fmt(end) + " s");
    }
    const double limit = s.cfl * sim.stability_limit();
    double dt = s.dt ? std::min(*s.dt, limit) : limit;
    const bool reduced = s.dt && *s.dt > limit;
    const double frame_time = std::min(end, static_cast<double>(next_frame) * s.frame_interval);
    if (!std::isfinite(dt)) dt = frame_time - sim.time();
    dt = std::min(dt, frame_time - sim.time());
    if (frame_time - (sim.time() + dt) < kTimeEps * std::max(1.0, frame_time)) dt = frame_time - sim.time();

    StepStats st = sim.advance(dt);
    st.dt_limited_by_stability = reduced;
    out.dt_reductions += reduced ? 1 : 0;
    out.history.push_back(st);
    if (on_step) on_step(st);

    if (sim.time() >= frame_time * (1.0 - kTimeEps)) {
      on_frame(out.frames, sim);
      ++out.frames;
      out.frame_times.push_back(sim.time());
      ++next_frame;
    }
    if (std::chrono::duration<double>(Clock::now() - start).count() > s.max_wall_seconds &&
        sim.time() < end * (1.0 - kTimeEps)) {
      out.status = "stopped";
      out.stopped_reason = "wall_time_budget";
      break;
    }
  }
  out.steps = sim.steps();
  out.time = sim.time();
  out.solver_requested = s.solver;
  out.solver_selected = resolve_solver(s.solver, sim.projector().system().unknowns());
  for (std::size_t n = 1; n < out.history.size(); ++n) out.pressure_iterations += out.history[n].iterations;
  if (const Multigrid* mg = sim.projector().multigrid()) {
    out.multigrid_levels = mg->levels();
    out.multigrid_memory_bytes = mg->memory_bytes();
  }
  out.multigrid_setups = sim.projector().multigrid_setups();
  for (const auto& st : out.history) {
    out.multigrid_setup_ms += st.setup_ms;
    out.fallbacks += st.fallback ? 1 : 0;
    out.solve_seconds += st.solve_ms / 1000.0;
  }
  out.fallback_reason = sim.fallback_reason();
  out.wall_seconds = std::chrono::duration<double>(Clock::now() - start).count();
  return out;
}

}  // namespace aeroflow
