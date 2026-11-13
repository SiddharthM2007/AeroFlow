#include "aeroflow/pressure.hpp"

#include "aeroflow/multigrid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

namespace aeroflow {

namespace {

using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }

constexpr std::uint8_t kXm = 1, kXp = 2, kYm = 4, kYp = 8, kZm = 16, kZp = 32;

double dot(std::span<const double> a, std::span<const double> b) {
  double s = 0.0;
  for (std::size_t n = 0; n < a.size(); ++n) s += a[n] * b[n];
  return s;
}

std::string fmt(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.3e", v);
  return buf;
}

}  // namespace

// --- settings -------------------------------------------------------------------------------

std::string_view to_string(PressureSolverKind kind) noexcept {
  switch (kind) {
    case PressureSolverKind::Auto: return "auto";
    case PressureSolverKind::Mgpcg: return "mgpcg";
    case PressureSolverKind::JacobiPcg: break;
  }
  return "jacobi_pcg";
}

std::optional<PressureSolverKind> parse_pressure_solver(std::string_view name) noexcept {
  if (name == "auto") return PressureSolverKind::Auto;
  if (name == "mgpcg") return PressureSolverKind::Mgpcg;
  if (name == "jacobi_pcg") return PressureSolverKind::JacobiPcg;
  return std::nullopt;
}

PressureSolverKind resolve_solver(PressureSolverKind requested, std::size_t unknowns) noexcept {
  if (requested != PressureSolverKind::Auto) return requested;
  return unknowns >= kAutoMultigridMinUnknowns ? PressureSolverKind::Mgpcg : PressureSolverKind::JacobiPcg;
}

void validate_projection_settings(const ProjectionSettings& s) {
  if (!std::isfinite(s.density) || s.density <= 0.0) {
    throw ProjectionError("invalid_settings", "density must be a finite number > 0 kg/m^3");
  }
  if (s.dt && (!std::isfinite(*s.dt) || *s.dt <= 0.0)) {
    throw ProjectionError("invalid_settings", "dt must be a finite number > 0 s");
  }
  if (!std::isfinite(s.tolerance) || s.tolerance <= 0.0 || s.tolerance >= 1.0) {
    throw ProjectionError("invalid_settings", "tolerance must lie in (0, 1)");
  }
  if (s.max_iterations == 0) throw ProjectionError("invalid_settings", "max_iterations must be at least 1");
  if (s.max_divergence && (!std::isfinite(*s.max_divergence) || *s.max_divergence <= 0.0)) {
    throw ProjectionError("invalid_settings", "max_divergence must be a finite number > 0 1/s");
  }
}

double projection_timestep(const ProjectionSettings& s, const MacGrid& grid, double inlet_velocity) {
  if (s.dt) return *s.dt;
  if (!(inlet_velocity > 0.0)) {
    throw ProjectionError("invalid_settings", "dt must be given explicitly when the inlet velocity is 0");
  }
  return 0.5 * std::min({grid.dx(), grid.dy(), grid.dz()}) / inlet_velocity;  // CFL 0.5
}

// --- PressureSystem -------------------------------------------------------------------------

PressureSystem::PressureSystem(const Grid3D& mask, const BoundaryMap& map)
    : nx_(mask.nx()), ny_(mask.ny()), nz_(mask.nz()), dx_(mask.dx()), dy_(mask.dy()), dz_(mask.dz()), map_(map) {
  if (map.nx != nx_ || map.ny != ny_ || map.nz != nz_) {
    throw BoundaryError("boundary map does not match the solid mask");
  }
  idx2_ = 1.0 / (dx_ * dx_);
  idy2_ = 1.0 / (dy_ * dy_);
  idz2_ = 1.0 / (dz_ * dz_);

  const auto cells = mask.cells();
  unknown_of_cell_.assign(cells.size(), -1);
  for (std::size_t c = 0; c < cells.size(); ++c) {
    if (cells[c] == CellType::Fluid) {
      unknown_of_cell_[c] = static_cast<std::int32_t>(cell_of_unknown_.size());
      cell_of_unknown_.push_back(static_cast<std::uint32_t>(c));
    }
  }

  const std::size_t n = cell_of_unknown_.size();
  coupled_.assign(n, 0);
  diag_.assign(n, 0.0);
  inv_diag_.assign(n, 0.0);
  std::vector<std::uint8_t> outlet(n, 0);
  const std::size_t nu0 = nx_ + 1, nvy = ny_ + 1;
  for (std::size_t u = 0; u < n; ++u) {
    const std::size_t c = cell_of_unknown_[u];
    const std::size_t i = c % nx_, j = (c / nx_) % ny_, k = c / (nx_ * ny_);
    const VelocityFaceBC f[6] = {
        map.u[i + nu0 * (j + ny_ * k)],      map.u[i + 1 + nu0 * (j + ny_ * k)],
        map.v[i + nx_ * (j + nvy * k)],      map.v[i + nx_ * (j + 1 + nvy * k)],
        map.w[i + nx_ * (j + ny_ * k)],      map.w[i + nx_ * (j + ny_ * (k + 1))],
    };
    const double h2[6] = {idx2_, idx2_, idy2_, idy2_, idz2_, idz2_};
    double diag = 0.0;
    for (int s = 0; s < 6; ++s) {
      if (f[s] == VelocityFaceBC::Interior) {
        coupled_[u] |= static_cast<std::uint8_t>(1u << s);
        diag += h2[s];
      } else if (f[s] == VelocityFaceBC::Outlet) {
        diag += 2.0 * h2[s];  // Dirichlet p = 0 on the face, half a cell away
        outlet[u] = 1;
      }
    }
    // A fluid cell with no coupled or outlet face is an isolated pocket: A_cc = 0. Give it a
    // unit diagonal for the preconditioner; its own (single-cell) component is singular.
    diag_[u] = diag;
    inv_diag_[u] = diag > 0.0 ? 1.0 / diag : 1.0;
  }

  // Components through coupled faces (iterative flood fill, no recursion).
  constexpr std::uint32_t kUnset = std::numeric_limits<std::uint32_t>::max();
  component_of_unknown_.assign(n, kUnset);
  std::vector<std::uint32_t> stack;
  const std::ptrdiff_t step[6] = {-1, 1, -static_cast<std::ptrdiff_t>(nx_), static_cast<std::ptrdiff_t>(nx_),
                                  -static_cast<std::ptrdiff_t>(nx_ * ny_), static_cast<std::ptrdiff_t>(nx_ * ny_)};
  for (std::size_t seed = 0; seed < n; ++seed) {
    if (component_of_unknown_[seed] != kUnset) continue;
    const auto id = static_cast<std::uint32_t>(components_.size());
    FluidComponent comp;
    comp.first_cell = cell_of_unknown_[seed];
    stack.push_back(static_cast<std::uint32_t>(seed));
    component_of_unknown_[seed] = id;
    while (!stack.empty()) {
      const std::uint32_t u = stack.back();
      stack.pop_back();
      ++comp.cells;
      comp.has_outlet = comp.has_outlet || outlet[u];
      const std::size_t c = cell_of_unknown_[u];
      if (c % nx_ == 0 && map.u[nu0 * (c / nx_)] == VelocityFaceBC::Inlet) comp.has_inlet = true;
      for (int s = 0; s < 6; ++s) {
        if (!(coupled_[u] & (1u << s))) continue;
        const auto nb = static_cast<std::uint32_t>(unknown_of_cell_[static_cast<std::size_t>(static_cast<std::ptrdiff_t>(c) + step[s])]);
        if (component_of_unknown_[nb] == kUnset) {
          component_of_unknown_[nb] = id;
          stack.push_back(nb);
        }
      }
    }
    components_.push_back(comp);
  }
}

std::size_t PressureSystem::memory_bytes() const noexcept {
  return unknown_of_cell_.size() * sizeof(std::int32_t) + cell_of_unknown_.size() * sizeof(std::uint32_t) +
         coupled_.size() + (diag_.size() + inv_diag_.size()) * sizeof(double) +
         component_of_unknown_.size() * sizeof(std::uint32_t);
}

void PressureSystem::apply(std::span<const double> x, std::span<double> y) const {
  const std::size_t n = cell_of_unknown_.size();
  const auto sx = static_cast<std::ptrdiff_t>(1), sy = static_cast<std::ptrdiff_t>(nx_),
             sz = static_cast<std::ptrdiff_t>(nx_ * ny_);
  for (std::size_t u = 0; u < n; ++u) {
    const auto c = static_cast<std::ptrdiff_t>(cell_of_unknown_[u]);
    const std::uint8_t f = coupled_[u];
    double off = 0.0;
    if (f & kXm) off += idx2_ * x[static_cast<std::size_t>(unknown_of_cell_[static_cast<std::size_t>(c - sx)])];
    if (f & kXp) off += idx2_ * x[static_cast<std::size_t>(unknown_of_cell_[static_cast<std::size_t>(c + sx)])];
    if (f & kYm) off += idy2_ * x[static_cast<std::size_t>(unknown_of_cell_[static_cast<std::size_t>(c - sy)])];
    if (f & kYp) off += idy2_ * x[static_cast<std::size_t>(unknown_of_cell_[static_cast<std::size_t>(c + sy)])];
    if (f & kZm) off += idz2_ * x[static_cast<std::size_t>(unknown_of_cell_[static_cast<std::size_t>(c - sz)])];
    if (f & kZp) off += idz2_ * x[static_cast<std::size_t>(unknown_of_cell_[static_cast<std::size_t>(c + sz)])];
    y[u] = diag_[u] * x[u] - off;
  }
}

void PressureSystem::build_rhs(const MacGrid& mac, std::span<double> b) const {
  const auto& u = mac.u();
  const auto& v = mac.v();
  const auto& w = mac.w();
  for (std::size_t n = 0; n < cell_of_unknown_.size(); ++n) {
    const std::size_t c = cell_of_unknown_[n];
    const std::size_t i = c % nx_, j = (c / nx_) % ny_, k = c / (nx_ * ny_);
    const double div = (u(i + 1, j, k) - u(i, j, k)) / dx_ + (v(i, j + 1, k) - v(i, j, k)) / dy_ +
                       (w(i, j, k + 1) - w(i, j, k)) / dz_;
    b[n] = -div;
  }
}

void PressureSystem::correct_velocity(MacGrid& mac, std::span<const double> phi) const {
  auto& u = mac.u();
  auto& v = mac.v();
  auto& w = mac.w();
  auto at = [&](std::size_t i, std::size_t j, std::size_t k) {
    return phi[static_cast<std::size_t>(unknown_of_cell_[i + nx_ * (j + ny_ * k)])];
  };
  const std::size_t nu0 = nx_ + 1, nvy = ny_ + 1;
  for (std::size_t k = 0; k < nz_; ++k)
    for (std::size_t j = 0; j < ny_; ++j)
      for (std::size_t i = 0; i <= nx_; ++i) {
        const VelocityFaceBC bc = map_.u[i + nu0 * (j + ny_ * k)];
        if (bc == VelocityFaceBC::Interior) {
          u(i, j, k) -= (at(i, j, k) - at(i - 1, j, k)) / dx_;
        } else if (bc == VelocityFaceBC::Outlet) {
          // Ghost phi_g = -phi_L places p = 0 on the outlet plane: (phi_g - phi_L)/dx = -2 phi_L/dx.
          u(i, j, k) -= -2.0 * at(i - 1, j, k) / dx_;
        }
      }
  for (std::size_t k = 0; k < nz_; ++k)
    for (std::size_t j = 0; j <= ny_; ++j)
      for (std::size_t i = 0; i < nx_; ++i)
        if (map_.v[i + nx_ * (j + nvy * k)] == VelocityFaceBC::Interior) {
          v(i, j, k) -= (at(i, j, k) - at(i, j - 1, k)) / dy_;
        }
  for (std::size_t k = 0; k <= nz_; ++k)
    for (std::size_t j = 0; j < ny_; ++j)
      for (std::size_t i = 0; i < nx_; ++i)
        if (map_.w[i + nx_ * (j + ny_ * k)] == VelocityFaceBC::Interior) {
          w(i, j, k) -= (at(i, j, k) - at(i, j, k - 1)) / dz_;
        }
}

// --- solver ---------------------------------------------------------------------------------

SolveStats solve_pressure(const PressureSystem& sys, std::span<const double> b_in, std::span<double> phi,
                          const ProjectionSettings& settings) {
  PcgWorkspace workspace;
  return solve_pressure(sys, b_in, phi, settings, workspace, SolveOptions{});
}

SolveStats solve_pressure(const PressureSystem& sys, std::span<const double> b_in, std::span<double> phi,
                          const ProjectionSettings& settings, PcgWorkspace& ws, const SolveOptions& options) {
  validate_projection_settings(settings);
  SolveStats stats;
  stats.requested = settings.solver;
  stats.selected = resolve_solver(settings.solver, sys.unknowns());

  // Preconditioner: Jacobi (pc == nullptr) or a multigrid V-cycle, prebuilt or built here.
  PressurePreconditioner* pc = nullptr;
  std::unique_ptr<Multigrid> local;
  auto fall_back = [&](std::string reason) {
    pc = nullptr;
    stats.fallback = true;
    stats.fallback_reason = std::move(reason);
  };
  if (stats.selected == PressureSolverKind::Mgpcg) {
    if (options.preconditioner) {
      pc = options.preconditioner;
      stats.multigrid_levels = options.preconditioner_levels;
    } else if (!options.preconditioner_unavailable.empty()) {
      fall_back(std::string(options.preconditioner_unavailable));
    } else {
      const auto setup_start = Clock::now();
      try {
        local = std::make_unique<Multigrid>(sys, settings.multigrid);
        pc = local.get();
        stats.multigrid_levels = local->levels();
      } catch (const std::exception& e) {
        fall_back(std::string("multigrid setup failed: ") + e.what());
      }
      stats.setup_ms = ms_since(setup_start);
    }
  }
  const auto solve_start = Clock::now();
  const std::size_t n = sys.unknowns();
  if (b_in.size() != n || phi.size() != n) throw std::invalid_argument("solve_pressure: vector sizes differ");
  const auto& comps = sys.components();
  ws.b.resize(n);
  ws.r.resize(n);
  ws.z.resize(n);
  ws.d.resize(n);
  ws.Ad.resize(n);
  ws.component_sum.assign(comps.size(), 0.0);
  ws.component_abs.assign(comps.size(), 0.0);
  auto& b = ws.b;
  auto& r = ws.r;
  auto& z = ws.z;
  auto& d = ws.d;
  auto& Ad = ws.Ad;
  auto& sum = ws.component_sum;
  auto& sum_abs = ws.component_abs;

  // Singular components (no outlet): check compatibility, then remove the (rounding-level) mean.
  std::copy(b_in.begin(), b_in.end(), b.begin());
  for (std::size_t u = 0; u < n; ++u) {
    sum[sys.component_of(u)] += b[u];
    sum_abs[sys.component_of(u)] += std::abs(b[u]);
  }
  for (std::size_t c = 0; c < comps.size(); ++c) {
    if (comps[c].has_outlet) continue;
    const double scale = sum_abs[c] + std::numeric_limits<double>::min();
    if (std::abs(sum[c]) > 1e-10 * scale && std::abs(sum[c]) > 1e-300) {
      throw ProjectionError(
          "incompatible_component",
          "fluid region of " + std::to_string(comps[c].cells) + " cells (first cell index " +
              std::to_string(comps[c].first_cell) + (comps[c].has_inlet ? ", fed by the inlet" : "") +
              ") has no outlet but a net source of " + fmt(sum[c]) +
              " (sum of -div): inflow cannot leave, so no divergence-free velocity exists");
    }
  }
  // Remove the (rounding-level) mean of each singular component in one pass.
  for (std::size_t u = 0; u < n; ++u) {
    const auto c = sys.component_of(u);
    if (!comps[c].has_outlet) b[u] -= sum[c] / static_cast<double>(comps[c].cells);
  }

  stats.rhs_norm = std::sqrt(dot(b, b));
  if (stats.rhs_norm == 0.0) {
    std::fill(phi.begin(), phi.end(), 0.0);
    stats.converged = true;
    stats.used = pc ? PressureSolverKind::Mgpcg : PressureSolverKind::JacobiPcg;
    return stats;
  }
  const double target = settings.tolerance * stats.rhs_norm;
  if (options.record_history) stats.history.reserve(std::min<std::size_t>(settings.max_iterations, 100000));

  // Initial residual: r = b (cold start, phi = 0) or b - A phi (warm start).
  if (options.initial_guess) {
    sys.apply(phi, Ad);
    for (std::size_t u = 0; u < n; ++u) r[u] = b[u] - Ad[u];
  } else {
    std::fill(phi.begin(), phi.end(), 0.0);
    std::copy(b.begin(), b.end(), r.begin());
  }

  const auto inv_diag = sys.inverse_diagonal();
  // Multigrid: z = P B P r, where P removes the mean of every enclosed region (the nullspace of A,
  // including isolated cells). r is in the range of A up to rounding; projecting both sides keeps the
  // preconditioner symmetric and keeps z and the search directions in the range of A. Jacobi keeps
  // its validated Milestone 5 form unchanged.
  bool singular = false;
  for (const auto& c : comps) singular = singular || !c.has_outlet;
  auto project_range = [&](std::span<double> v) {
    if (!singular) return;
    std::fill(sum.begin(), sum.end(), 0.0);
    for (std::size_t u = 0; u < n; ++u)
      if (!comps[sys.component_of(u)].has_outlet) sum[sys.component_of(u)] += v[u];
    for (std::size_t u = 0; u < n; ++u) {
      const auto c = sys.component_of(u);
      if (!comps[c].has_outlet) v[u] -= sum[c] / static_cast<double>(comps[c].cells);
    }
  };
  if (pc && singular) ws.projected.resize(n);
  auto precondition = [&] {
    if (!pc) {
      for (std::size_t u = 0; u < n; ++u) z[u] = inv_diag[u] * r[u];
      return;
    }
    if (singular) {
      std::copy(r.begin(), r.end(), ws.projected.begin());
      project_range(ws.projected);
      pc->apply(ws.projected, z);
    } else {
      pc->apply(r, z);
    }
    project_range(z);
  };
  // A preconditioner that is not numerically positive definite ends the multigrid phase.
  auto lost_positivity = [&](const char* what, double value) {
    fall_back(std::string("multigrid preconditioner lost positivity (") + what + " = " + fmt(value) +
              ") at iteration " + std::to_string(stats.iterations));
  };
  std::size_t it = 0;
  double rnorm = std::sqrt(dot(r, r));
  auto max_abs = [&](std::span<const double> v) {
    double m = 0.0;
    for (const double x : v) m = std::max(m, std::abs(x));
    return m;
  };
  // Stopping test: the relative 2-norm, plus the optional max-norm bound (evaluated only once the
  // 2-norm test passes, so the default path is unchanged).
  auto met = [&] { return rnorm <= target && (!settings.max_divergence || max_abs(r) <= *settings.max_divergence); };
  while (true) {
    // (Re)start CG from the current residual.
    precondition();
    std::copy(z.begin(), z.end(), d.begin());
    double rz = dot(r, z);
    bool restart = false;
    if (pc && !met() && !(std::isfinite(rz) && rz > 0.0)) {
      stats.iterations = it;
      stats.fallback_iteration = it;
      lost_positivity("r.z", rz);
      continue;  // restart with Jacobi from the same iterate
    }
    while (it < settings.max_iterations && !met()) {
      sys.apply(d, Ad);
      const double dAd = dot(d, Ad);
      if (!(dAd > 0.0)) {
        if (pc) {
          stats.iterations = it;
          stats.fallback_iteration = it;
          lost_positivity("d.Ad", dAd);
          restart = true;
          break;
        }
        throw ProjectionError("solver_breakdown", "conjugate gradient breakdown (d^T A d = " + fmt(dAd) +
                                                      ") at iteration " + std::to_string(it));
      }
      const double alpha = rz / dAd;
      for (std::size_t u = 0; u < n; ++u) {
        phi[u] += alpha * d[u];
        r[u] -= alpha * Ad[u];
      }
      ++it;
      rnorm = std::sqrt(dot(r, r));
      if (options.record_history) stats.history.push_back(rnorm / stats.rhs_norm);
      if (met()) break;
      precondition();
      const double rz_new = dot(r, z);
      if (pc && !(std::isfinite(rz_new) && rz_new > 0.0)) {
        stats.iterations = it;
        stats.fallback_iteration = it;
        lost_positivity("r.z", rz_new);
        restart = true;
        break;
      }
      const double beta = rz_new / rz;
      rz = rz_new;
      for (std::size_t u = 0; u < n; ++u) d[u] = z[u] + beta * d[u];
    }
    (void)restart;  // a fallback restarts below from the true residual, now with Jacobi
    // Verify with the true residual (recursive residuals drift); restart if it is not met.
    sys.apply(phi, Ad);
    for (std::size_t u = 0; u < n; ++u) r[u] = b[u] - Ad[u];
    rnorm = std::sqrt(dot(r, r));
    stats.residual_norm = rnorm;
    stats.max_residual = max_abs(r);
    if (met()) {
      stats.converged = true;
      break;
    }
    if (it >= settings.max_iterations) break;
  }
  stats.iterations = it;
  stats.relative_residual = stats.residual_norm / stats.rhs_norm;
  stats.used = pc ? PressureSolverKind::Mgpcg : PressureSolverKind::JacobiPcg;
  stats.solve_ms = ms_since(solve_start);

  // Zero-mean gauge on singular components (two passes over the unknowns).
  std::fill(sum.begin(), sum.end(), 0.0);
  for (std::size_t u = 0; u < n; ++u)
    if (!comps[sys.component_of(u)].has_outlet) sum[sys.component_of(u)] += phi[u];
  for (std::size_t u = 0; u < n; ++u) {
    const auto c = sys.component_of(u);
    if (!comps[c].has_outlet) phi[u] -= sum[c] / static_cast<double>(comps[c].cells);
  }

  if (!stats.converged) {
    throw ProjectionError("projection_not_converged",
                          "pressure solver did not converge: relative residual " + fmt(stats.relative_residual) +
                              " after " + std::to_string(stats.iterations) + " iterations (tolerance " +
                              fmt(settings.tolerance) + ")");
  }
  return stats;
}

// --- projection -----------------------------------------------------------------------------

ProjectionResult project_velocity(MacGrid& mac, const Grid3D& mask, const BoundaryMap& map,
                                  const BoundaryConditions& bc, const ProjectionSettings& settings) {
  const auto start = Clock::now();
  validate_projection_settings(settings);
  if (!mac.matches(mask)) throw BoundaryError("solid mask does not match the MAC grid");
  ProjectionResult res;
  res.density = settings.density;
  res.dt = projection_timestep(settings, mac, bc.inlet_velocity);
  res.inlet_velocity = bc.inlet_velocity;

  apply_velocity_bcs(mac, map, bc);
  res.divergence_before = compute_divergence(mac, mask);
  res.before_stats = divergence_stats(res.divergence_before, mask);

  const PressureSystem sys(mask, map);
  std::vector<double> b(sys.unknowns()), phi(sys.unknowns());
  sys.build_rhs(mac, b);
  res.components = sys.components().size();
  for (const auto& c : sys.components()) res.enclosed_components += c.has_outlet ? 0 : 1;
  res.setup_ms = ms_since(start);

  const auto solve_start = Clock::now();
  res.solve = solve_pressure(sys, b, phi, settings);
  res.solve_ms = ms_since(solve_start);

  sys.correct_velocity(mac, phi);
  apply_prescribed_velocity_bcs(mac, map, bc);  // no-op for a correct correction; no outlet copy

  auto& p = mac.p();
  p.fill(0.0);
  const double scale = res.density / res.dt;
  for (std::size_t u = 0; u < sys.unknowns(); ++u) p.data()[sys.cell_of_unknown(u)] = scale * phi[u];

  res.divergence_after = compute_divergence(mac, mask);
  res.after_stats = divergence_stats(res.divergence_after, mask);
  res.fluxes = boundary_fluxes(mac, map);
  res.memory_bytes = sys.memory_bytes() + (b.size() + phi.size() + 4 * sys.unknowns()) * sizeof(double) +
                     MacGrid::required_bytes(mac.nx(), mac.ny(), mac.nz());
  res.total_ms = ms_since(start);
  return res;
}

StaggeredArray velocity_magnitude(const MacGrid& mac, const Grid3D& mask) {
  if (!mac.matches(mask)) throw BoundaryError("solid mask does not match the MAC grid");
  StaggeredArray out(mac.nx(), mac.ny(), mac.nz(), 0.0);
  const auto cells = mask.cells();
  for (std::size_t k = 0; k < mac.nz(); ++k)
    for (std::size_t j = 0; j < mac.ny(); ++j)
      for (std::size_t i = 0; i < mac.nx(); ++i) {
        if (cells[mask.index(i, j, k)] != CellType::Fluid) continue;
        const double ub = 0.5 * (mac.u()(i, j, k) + mac.u()(i + 1, j, k));
        const double vb = 0.5 * (mac.v()(i, j, k) + mac.v()(i, j + 1, k));
        const double wb = 0.5 * (mac.w()(i, j, k) + mac.w()(i, j, k + 1));
        out(i, j, k) = std::sqrt(ub * ub + vb * vb + wb * wb);
      }
  return out;
}

// --- Projector ------------------------------------------------------------------------------

Projector::Projector(const Grid3D& mask, const BoundaryMap& map)
    : mask_(mask), map_(map), system_(mask, map), b_(system_.unknowns()), phi_(system_.unknowns()) {}

Projector::~Projector() = default;

std::size_t Projector::memory_bytes() const noexcept {
  const std::size_t vectors = b_.size() + phi_.size() + workspace_.b.size() + workspace_.r.size() +
                              workspace_.z.size() + workspace_.d.size() + workspace_.Ad.size();
  return system_.memory_bytes() + vectors * sizeof(double) + workspace_.projected.size() * sizeof(double) +
         (before_.values.size() + after_.values.size()) * sizeof(double) + (multigrid_ ? multigrid_->memory_bytes() : 0);
}

Projector::Result Projector::project(MacGrid& mac, const BoundaryConditions& bc, const ProjectionSettings& settings,
                                     double dt) {
  validate_projection_settings(settings);
  if (!(dt > 0.0) || !std::isfinite(dt)) throw ProjectionError("invalid_settings", "dt must be a finite number > 0 s");
  if (!mac.matches(mask_)) throw BoundaryError("solid mask does not match the MAC grid");
  Result res;
  apply_velocity_bcs(mac, map_, bc);
  compute_divergence_into(mac, mask_, before_);
  res.before = divergence_stats(before_, mask_);

  system_.build_rhs(mac, b_);
  SolveOptions options;
  options.initial_guess = has_previous_;
  options.record_history = false;
  if (has_previous_ && dt != previous_dt_) {
    // Carry the pressure over, not the potential: phi = (dt/rho) p.
    const double scale = dt / previous_dt_;
    for (double& x : phi_) x *= scale;
  }
  res.warm_started = has_previous_;
  // The multigrid hierarchy depends only on the grid and obstacles: build it once, then reuse it
  // (and its work buffers) for every later step. A failed setup is reported, never retried.
  double setup_ms = 0.0;
  if (resolve_solver(settings.solver, system_.unknowns()) == PressureSolverKind::Mgpcg) {
    if (!multigrid_ && multigrid_error_.empty()) {
      const auto start = std::chrono::steady_clock::now();
      try {
        multigrid_ = std::make_unique<Multigrid>(mask_, map_, system_, settings.multigrid);
        ++multigrid_setups_;
      } catch (const std::exception& e) {
        multigrid_error_ = std::string("multigrid setup failed: ") + e.what();
      }
      setup_ms = ms_since(start);
    }
    options.preconditioner = multigrid_.get();
    options.preconditioner_levels = multigrid_ ? multigrid_->levels() : 0;
    options.preconditioner_unavailable = multigrid_error_;
  }
  res.solve = solve_pressure(system_, b_, phi_, settings, workspace_, options);
  res.solve.setup_ms += setup_ms;
  has_previous_ = true;
  previous_dt_ = dt;

  system_.correct_velocity(mac, phi_);
  apply_prescribed_velocity_bcs(mac, map_, bc);

  auto p = mac.p().data();
  std::fill(p.begin(), p.end(), 0.0);
  const double scale = settings.density / dt;
  for (std::size_t u = 0; u < system_.unknowns(); ++u) p[system_.cell_of_unknown(u)] = scale * phi_[u];

  compute_divergence_into(mac, mask_, after_);
  res.after = divergence_stats(after_, mask_);
  res.fluxes = boundary_fluxes(mac, map_);
  return res;
}

}  // namespace aeroflow
