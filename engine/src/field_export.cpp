#include "aeroflow/field_export.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <stdexcept>

#include "aeroflow/boundary.hpp"
#include "aeroflow/divergence.hpp"
#include "aeroflow/mac_grid.hpp"
#include "aeroflow/simulation.hpp"

namespace aeroflow {

namespace {

ScalarField cell_field(const Grid3D& mask, std::string name, std::string quantity, std::string long_name,
                       std::string units, bool is_signed, std::span<const double> values,
                       nlohmann::json provenance) {
  ScalarField f;
  f.name = std::move(name);
  f.quantity = std::move(quantity);
  f.long_name = std::move(long_name);
  f.units = std::move(units);
  f.is_signed = is_signed;
  f.location = FieldLocation::Cell;
  f.shape = field_shape(FieldLocation::Cell, mask.nx(), mask.ny(), mask.nz());
  f.values.assign(values.begin(), values.end());
  f.valid.resize(mask.cell_count());
  const auto cells = mask.cells();
  for (std::size_t n = 0; n < cells.size(); ++n) f.valid[n] = cells[n] == CellType::Fluid ? 1 : 0;
  f.frame = {{"index", 0}, {"time_s", nullptr}, {"label", "initialized"}};
  f.mask = {{"source", "grid_classification"}, {"valid_rule", "fluid cells"}};
  f.provenance = std::move(provenance);
  return f;
}

std::string compact(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%g", v);
  return buf;
}

nlohmann::json divergence_json(const std::optional<DivergenceStats>& s) {
  if (!s) return nullptr;
  return {{"fluid_cells", s->fluid_cells}, {"max_abs", s->max_abs}, {"mean_abs", s->mean_abs}, {"rms", s->rms}};
}

}  // namespace

std::vector<std::string> known_field_ids() {
  return {kDivergenceDiagnostic, kPressure,     kDivergenceBefore, kDivergenceAfter, kVelocityMagnitude,
          kSimPressure,          kSimVelocityMagnitude, kSimDivergence, kSimU, kSimV, kSimW};
}

bool is_projection_field(const std::string& id) {
  return id == kPressure || id == kDivergenceBefore || id == kDivergenceAfter || id == kVelocityMagnitude;
}

ScalarField divergence_diagnostic(const Grid3D& mask, double inlet_velocity) {
  MacGrid mac(mask);
  const FaceTypes faces = classify_faces(mask);
  mac.set_uniform_velocity({inlet_velocity, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  const DivergenceField div = compute_divergence(mac, mask);
  return cell_field(
      mask, kDivergenceDiagnostic, "divergence", "Velocity divergence of the initialized field", "1/s", true,
      div.values.data(),
      {
          {"kind", "diagnostic"},
          {"solved", false},
          {"description",
           "Divergence of a prescribed initial field: uniform +x " + compact(inlet_velocity) +
               " m/s on active faces (tunnel walls included), 0 on faces blocked by solids. Not a CFD solution."},
          {"initial_velocity", {inlet_velocity, 0.0, 0.0}},
          {"blocked_faces", "normal velocity 0"},
      });
}

ProjectedFields project_initialized_field(const Grid3D& mask, const ProjectionConfig& config) {
  MacGrid mac(mask);
  const FaceTypes faces = classify_faces(mask);
  mac.set_uniform_velocity({config.inlet_velocity, 0.0, 0.0}, FaceSelection::ActiveOnly, &faces);
  const BoundaryMap map = classify_boundaries(mask);
  ProjectionSettings settings;
  settings.density = config.density;
  settings.dt = config.dt;
  settings.tolerance = config.tolerance;
  settings.max_iterations = config.max_iterations;
  settings.solver = config.pressure_solver;

  ProjectedFields out;
  out.result = project_velocity(mac, mask, map, BoundaryConditions{config.inlet_velocity}, settings);
  const auto& r = out.result;
  const nlohmann::json provenance = {
      {"kind", "projection"},
      {"solved", false},
      {"description",
       "One pressure projection of the initialized field (uniform +x inlet velocity on active faces, "
       "velocity boundary conditions applied). Approximately divergence-free to the solver tolerance; "
       "not a time-accurate Navier-Stokes solution."},
      {"inlet_velocity", config.inlet_velocity},
      {"density", r.density},
      {"dt", r.dt},
      {"iterations", r.solve.iterations},
      {"relative_residual", r.solve.relative_residual},
      {"tolerance", config.tolerance},
  };
  const auto magnitude = velocity_magnitude(mac, mask);
  out.pressure = cell_field(mask, kPressure, "pressure", "Projection pressure (gauge, p = 0 at outlet)", "Pa", true,
                            mac.p().data(), provenance);
  out.divergence_before = cell_field(mask, kDivergenceBefore, "divergence", "Divergence before projection", "1/s",
                                     true, r.divergence_before.values.data(), provenance);
  out.divergence_after = cell_field(mask, kDivergenceAfter, "divergence", "Divergence after projection", "1/s",
                                    true, r.divergence_after.values.data(), provenance);
  out.velocity_magnitude = cell_field(mask, kVelocityMagnitude, "velocity_magnitude",
                                      "Velocity magnitude after projection", "m/s", false, magnitude.data(),
                                      provenance);
  return out;
}

nlohmann::json projection_summary(const ProjectionResult& r, const ProjectionConfig& config) {
  return {
      {"inlet_velocity", config.inlet_velocity},
      {"density", r.density},
      {"dt", r.dt},
      {"dt_source", config.dt ? "explicit" : "cfl_0.5"},
      {"tolerance", config.tolerance},
      {"max_iterations", config.max_iterations},
      {"iterations", r.solve.iterations},
      {"rhs_norm", r.solve.rhs_norm},
      {"relative_residual", r.solve.relative_residual},
      {"converged", r.solve.converged},
      {"components", r.components},
      {"enclosed_components", r.enclosed_components},
      {"divergence_before", divergence_json(r.before_stats)},
      {"divergence_after", divergence_json(r.after_stats)},
      {"fluxes",
       {{"inlet", r.fluxes.inlet},
        {"outlet", r.fluxes.outlet},
        {"wall_leak", r.fluxes.wall_leak},
        {"inlet_area", r.fluxes.inlet_area},
        {"outlet_area", r.fluxes.outlet_area}}},
      {"memory_bytes", r.memory_bytes},
      {"setup_ms", r.setup_ms},
      {"solve_ms", r.solve_ms},
      {"total_ms", r.total_ms},
      {"pressure_solver",
       {{"requested", to_string(r.solve.requested)},
        {"selected", to_string(r.solve.selected)},
        {"used", to_string(r.solve.used)},
        {"multigrid_levels", r.solve.multigrid_levels},
        {"multigrid_setup_ms", r.solve.setup_ms},
        {"pcg_ms", r.solve.solve_ms},
        {"fallback", r.solve.fallback},
        {"fallback_reason", r.solve.fallback ? nlohmann::json(r.solve.fallback_reason) : nlohmann::json(nullptr)}}},
  };
}

namespace {

nlohmann::json summary_of(const nlohmann::json& header, const std::string& file) {
  return {
      {"name", header["name"]},         {"file", file},
      {"quantity", header["quantity"]}, {"location", header["location"]},
      {"shape", header["shape"]},       {"units", header["units"]},
      {"frame", header["frame"]},       {"stats", header["stats"]},
      {"provenance", header["provenance"]},
  };
}

SimulationSettings to_settings(const SimulationConfig& c) {
  SimulationSettings s;
  s.inlet_velocity = c.inlet_velocity;
  s.density = c.density;
  s.viscosity = c.viscosity;
  s.duration = c.duration;
  s.frame_interval = c.frame_interval;
  s.cfl = c.cfl;
  s.dt = c.dt;
  s.max_steps = c.max_steps;
  s.tolerance = c.tolerance;
  s.max_iterations = c.max_iterations;
  s.initial = c.initial;
  s.max_wall_seconds = c.max_wall_seconds;
  s.solver = c.pressure_solver;
  return s;
}

const char* kSchemeDescription =
    "Incompressible Navier-Stokes, fractional-step projection (non-incremental Chorin): first-order "
    "upwind advection (advective form), explicit 7-point viscous diffusion, forward Euler with "
    "adaptive dt (CFL), matrix-free preconditioned conjugate-gradient pressure Poisson solve (Jacobi-PCG or "
    "MG-PCG: same equations and tolerance; see pressure_solver). Pressure p = rho phi / dt is a "
    "first-order (in dt) approximation of the physical gauge pressure (p = 0 on the outlet plane). "
    "First-order upwinding adds numerical viscosity ~ |U| h / 2, so results are grid-dependent.";

// Builds one simulation output field at the current state.
ScalarField simulation_field(const std::string& id, const Simulation& sim, std::size_t frame) {
  const Grid3D& mask = sim.mask();
  const MacGrid& mac = sim.state();
  const SimulationSettings& set = sim.settings();
  const nlohmann::json provenance = {
      {"kind", "simulation"},
      {"solved", true},
      {"description", kSchemeDescription},
      {"step", sim.steps()},
      {"time_s", sim.time()},
      {"inlet_velocity", set.inlet_velocity},
      {"viscosity", set.viscosity},
      {"density", set.density},
      {"cfl", set.cfl},
      {"tolerance", set.tolerance},
      {"pressure_solver", to_string(resolve_solver(set.solver, sim.projector().system().unknowns()))},
  };
  char label[48];
  std::snprintf(label, sizeof label, "t = %.4g s", sim.time());
  const nlohmann::json frame_info = {{"index", frame}, {"time_s", sim.time()}, {"label", label}};

  if (id == kSimU || id == kSimV || id == kSimW) {
    const int axis = id == kSimU ? 0 : id == kSimV ? 1 : 2;
    const StaggeredArray& f = axis == 0 ? mac.u() : axis == 1 ? mac.v() : mac.w();
    const auto& bcs = axis == 0 ? sim.boundaries().u : axis == 1 ? sim.boundaries().v : sim.boundaries().w;
    ScalarField out;
    out.name = id;
    out.quantity = axis == 0 ? "velocity_x" : axis == 1 ? "velocity_y" : "velocity_z";
    out.long_name = std::string("Velocity component ") + "uvw"[axis] + " (staggered faces)";
    out.units = "m/s";
    out.is_signed = true;
    out.location = axis == 0 ? FieldLocation::FaceX : axis == 1 ? FieldLocation::FaceY : FieldLocation::FaceZ;
    out.shape = field_shape(out.location, mask.nx(), mask.ny(), mask.nz());
    out.values.assign(f.data().begin(), f.data().end());
    out.valid.resize(bcs.size());
    for (std::size_t n = 0; n < bcs.size(); ++n) out.valid[n] = bcs[n] != VelocityFaceBC::Inactive ? 1 : 0;
    out.frame = frame_info;
    out.mask = {{"source", "boundary_map"}, {"valid_rule", "faces not inside solids"}};
    out.provenance = provenance;
    return out;
  }
  ScalarField out;
  if (id == kSimPressure) {
    out = cell_field(mask, id, "pressure", "Pressure (gauge, p = 0 at outlet)", "Pa", true, mac.p().data(), provenance);
  } else if (id == kSimDivergence) {
    out = cell_field(mask, id, "divergence", "Velocity divergence after projection", "1/s", true,
                     sim.divergence().values.data(), provenance);
  } else {
    const auto magnitude = velocity_magnitude(mac, mask);
    out = cell_field(mask, id, "velocity_magnitude", "Velocity magnitude", "m/s", false, magnitude.data(), provenance);
  }
  out.frame = frame_info;
  return out;
}

nlohmann::json simulation_summary(const RunSummary& r, const SimulationConfig& c, std::size_t memory_bytes) {
  nlohmann::json history;
  auto column = [&](const char* name, auto get) {
    nlohmann::json col = nlohmann::json::array();
    for (const auto& st : r.history) col.push_back(get(st));
    history[name] = std::move(col);
  };
  column("step", [](const StepStats& s) { return s.step; });
  column("time", [](const StepStats& s) { return s.time; });
  column("dt", [](const StepStats& s) { return s.dt; });
  column("iterations", [](const StepStats& s) { return s.iterations; });
  column("relative_residual", [](const StepStats& s) { return s.relative_residual; });
  column("div_before_max", [](const StepStats& s) { return s.div_before_max; });
  column("div_after_max", [](const StepStats& s) { return s.div_after_max; });
  column("div_after_rms", [](const StepStats& s) { return s.div_after_rms; });
  column("inflow", [](const StepStats& s) { return s.inflow; });
  column("outflow", [](const StepStats& s) { return s.outflow; });
  column("wall_leak", [](const StepStats& s) { return s.wall_leak; });
  column("max_speed", [](const StepStats& s) { return s.max_speed; });
  column("kinetic_energy", [](const StepStats& s) { return s.kinetic_energy; });
  column("solve_ms", [](const StepStats& s) { return s.solve_ms; });
  column("solver", [](const StepStats& s) { return std::string(to_string(s.solver)); });
  std::size_t max_it = 0, sum_it = 0;
  for (std::size_t n = 1; n < r.history.size(); ++n) {
    max_it = std::max(max_it, r.history[n].iterations);
    sum_it += r.history[n].iterations;
  }
  return {
      {"status", r.status},
      {"stopped_reason", r.stopped_reason.empty() ? nlohmann::json(nullptr) : nlohmann::json(r.stopped_reason)},
      {"steps", r.steps},
      {"time", r.time},
      {"frames", r.frames},
      {"frame_times", r.frame_times},
      {"wall_seconds", r.wall_seconds},
      {"dt_reductions", r.dt_reductions},
      {"memory_bytes", memory_bytes},
      {"mean_iterations", r.steps ? static_cast<double>(sum_it) / static_cast<double>(r.steps) : 0.0},
      {"max_iterations", max_it},
      {"settings",
       {{"inlet_velocity", c.inlet_velocity}, {"density", c.density}, {"viscosity", c.viscosity},
        {"duration", c.duration}, {"frame_interval", c.frame_interval}, {"cfl", c.cfl},
        {"dt", c.dt ? nlohmann::json(*c.dt) : nlohmann::json(nullptr)}, {"max_steps", c.max_steps},
        {"tolerance", c.tolerance}, {"max_iterations", c.max_iterations}, {"initial", c.initial},
        {"pressure_solver", to_string(c.pressure_solver)},
        {"max_wall_seconds", c.max_wall_seconds}}},
      {"scheme", kSchemeDescription},
      {"pressure_solver",
       {{"requested", to_string(r.solver_requested)},
        {"selected", to_string(r.solver_selected)},
        {"used", r.fallbacks ? "jacobi_pcg" : to_string(r.solver_selected)},
        {"pressure_iterations", r.pressure_iterations},
        {"multigrid_levels", r.multigrid_levels},
        {"multigrid_setups", r.multigrid_setups},
        {"multigrid_setup_ms", r.multigrid_setup_ms},
        {"multigrid_memory_bytes", r.multigrid_memory_bytes},
        {"solve_seconds", r.solve_seconds},
        {"fallbacks", r.fallbacks},
        {"fallback_reason", r.fallbacks ? nlohmann::json(r.fallback_reason) : nlohmann::json(nullptr)}}},
      {"history", history},
  };
}

}  // namespace

bool is_simulation_field(const std::string& id) {
  return id == kSimPressure || id == kSimVelocityMagnitude || id == kSimDivergence || id == kSimU || id == kSimV ||
         id == kSimW;
}

FieldExport export_fields(const Grid3D& mask, const std::vector<std::string>& requested,
                          const std::string& output_dir, const ProjectionConfig& projection,
                          const SimulationConfig& simulation, const ProgressCallback& progress) {
  FieldExport out;
  if (requested.empty()) return out;
  const auto known = known_field_ids();
  for (const auto& id : requested) {
    if (std::find(known.begin(), known.end(), id) == known.end()) throw std::invalid_argument("unknown field id: " + id);
  }
  if (!std::filesystem::is_directory(output_dir)) {
    throw FieldError("field output directory does not exist: " + output_dir);
  }

  // Run the (expensive) projection at most once, and before writing anything.
  std::optional<ProjectedFields> projected;
  if (std::any_of(requested.begin(), requested.end(), is_projection_field)) {
    projected = project_initialized_field(mask, projection);
    out.projection = projection_summary(projected->result, projection);
  }

  const MacGrid geometry(mask);  // spacing/extents for headers (same convention as Grid3D)
  for (const auto& id : requested) {
    if (is_simulation_field(id)) continue;
    ScalarField field;
    if (id == kDivergenceDiagnostic) field = divergence_diagnostic(mask, projection.inlet_velocity);
    else if (id == kPressure) field = projected->pressure;
    else if (id == kDivergenceBefore) field = projected->divergence_before;
    else if (id == kDivergenceAfter) field = projected->divergence_after;
    else field = projected->velocity_magnitude;
    const std::string file = field.name + ".f0.affield";
    const auto header = write_field_file((std::filesystem::path(output_dir) / file).string(), field, geometry);
    out.fields.push_back(summary_of(header, file));
  }

  std::vector<std::string> sim_ids;
  for (const auto& id : requested)
    if (is_simulation_field(id)) sim_ids.push_back(id);
  if (!sim_ids.empty()) {
    Simulation sim(mask, to_settings(simulation));
    using Clock = std::chrono::steady_clock;
    auto last_report = Clock::now() - std::chrono::seconds(1);
    std::size_t frames_written = 0;
    auto report = [&](const StepStats& st, bool force) {
      if (!progress) return;
      const auto now = Clock::now();
      if (!force && now - last_report < std::chrono::milliseconds(250)) return;
      last_report = now;
      progress({{"step", st.step},
                {"time", st.time},
                {"duration", simulation.duration},
                {"dt", st.dt},
                {"iterations", st.iterations},
                {"relative_residual", st.relative_residual},
                {"div_after_max", st.div_after_max},
                {"max_speed", st.max_speed},
                {"frames", frames_written}});
    };
    const auto summary = run_simulation(
        sim,
        [&](std::size_t frame, const Simulation& s) {
          for (const auto& id : sim_ids) {
            const ScalarField field = simulation_field(id, s, frame);
            const std::string file = id + ".f" + std::to_string(frame) + ".affield";
            const auto header = write_field_file((std::filesystem::path(output_dir) / file).string(), field, geometry);
            out.fields.push_back(summary_of(header, file));
          }
          ++frames_written;
        },
        [&](const StepStats& st) { report(st, false); });
    if (!summary.history.empty()) report(summary.history.back(), true);
    out.simulation = simulation_summary(summary, simulation, sim.memory_bytes());
  }
  return out;
}

}  // namespace aeroflow
