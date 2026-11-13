#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "aeroflow/boundary.hpp"
#include "aeroflow/divergence.hpp"
#include "aeroflow/grid3d.hpp"
#include "aeroflow/mac_grid.hpp"
#include "aeroflow/multigrid_settings.hpp"

namespace aeroflow {

// Pressure Poisson solve and MAC velocity projection (Milestone 5).
//
// Unknowns: one projection potential phi = (dt/rho) p per fluid cell (x-fastest numbering).
// Gradient on a face f between cells L and R (spacing h along the face normal):
//   Coupled  (Interior face): (G phi)_f = (phi_R - phi_L) / h
//   Dirichlet (Outlet face):  p = 0 on the outlet plane, ghost phi_g = -phi_L, so
//                             (G phi)_f = (phi_g - phi_L)/dx = -phi_L / (dx/2)   (half-cell distance)
//   Neumann  (prescribed faces): no correction, (G phi)_f = 0
// Projection: u_new = u* - G phi on Coupled and Outlet faces only.
// L = D o G (D = existing MAC divergence) is symmetric negative semidefinite; we solve the SPD
//   A phi = b,   A = -L,   b = -D u*,
//   (A phi)_c = sum_coupled (phi_c - phi_nb)/h^2 + sum_outlet 2 phi_c / dx^2
// so that D u_new = D u* - L phi = -(b - A phi): the remaining divergence is exactly the
// negated solver residual.

class ProjectionError : public std::runtime_error {
 public:
  // code: "invalid_settings", "incompatible_component", "projection_not_converged", "solver_breakdown"
  ProjectionError(std::string code, const std::string& message)
      : std::runtime_error(message), code_(std::move(code)) {}
  [[nodiscard]] const std::string& code() const noexcept { return code_; }

 private:
  std::string code_;
};

// Pressure solver (Milestone 7). Both are preconditioned conjugate gradients on the same A, b and
// stopping test; only the preconditioner differs:
//   JacobiPcg: z = D^-1 r (the validated Milestone 5 baseline)
//   Mgpcg:     z = P B P r, one symmetric multigrid V-cycle B (multigrid.hpp), with P the
//              Euclidean projection that removes the mean of each enclosed (no-outlet) region.
//   Auto:      Mgpcg when the grid has at least kAutoMultigridMinUnknowns fluid cells, else
//              JacobiPcg (measured in Milestone 7 phase 4: below ~1,000 cells Jacobi-PCG is
//              marginally faster, by less than 0.1 ms per solve; above it MG-PCG wins by a growing
//              factor, 2x at 3.5k cells and 8-19x per solve at 65k-2M cells).
enum class PressureSolverKind { Auto, JacobiPcg, Mgpcg };
inline constexpr std::size_t kAutoMultigridMinUnknowns = 1000;
[[nodiscard]] std::string_view to_string(PressureSolverKind kind) noexcept;
// "auto" | "mgpcg" | "jacobi_pcg"; nullopt for anything else.
[[nodiscard]] std::optional<PressureSolverKind> parse_pressure_solver(std::string_view name) noexcept;
// The concrete solver for a request (Auto resolved by the number of fluid unknowns).
[[nodiscard]] PressureSolverKind resolve_solver(PressureSolverKind requested, std::size_t unknowns) noexcept;

struct ProjectionSettings {
  double density = 1.225;            // kg/m^3, > 0
  std::optional<double> dt;          // s, > 0; default CFL: 0.5 * min(dx,dy,dz) / U_in
  double tolerance = 1e-10;          // relative residual ||b - A phi||_2 / ||b||_2, in (0, 1)
  std::size_t max_iterations = 10000;
  PressureSolverKind solver = PressureSolverKind::Auto;
  MultigridSettings multigrid;       // used when the (resolved) solver is Mgpcg
  // Optional extra stopping test (off by default): also require max_c |b - A phi|_c <= this value
  // (1/s). The residual equals minus the divergence left after projection, so this bounds the
  // largest per-cell divergence. The relative 2-norm test above always applies.
  std::optional<double> max_divergence;
};

// Throws ProjectionError("invalid_settings") for non-finite/non-positive density or dt, tolerance
// outside (0, 1), or zero iterations.
void validate_projection_settings(const ProjectionSettings& s);

// The timestep used to report pressure: explicit dt, else 0.5 * min spacing / U_in.
// Throws ProjectionError("invalid_settings") if dt is unset and U_in == 0.
double projection_timestep(const ProjectionSettings& s, const MacGrid& grid, double inlet_velocity);

struct FluidComponent {
  std::size_t cells = 0;
  std::size_t first_cell = 0;  // linear index of its first cell (x-fastest)
  bool has_outlet = false;     // has a Dirichlet face: A is SPD on it
  bool has_inlet = false;
};

// Matrix-free pressure operator on fluid cells. Built once per mask/boundary map.
class PressureSystem {
 public:
  PressureSystem(const Grid3D& mask, const BoundaryMap& map);

  [[nodiscard]] std::size_t unknowns() const noexcept { return cell_of_unknown_.size(); }
  [[nodiscard]] std::int32_t unknown_of_cell(std::size_t cell) const noexcept { return unknown_of_cell_[cell]; }
  [[nodiscard]] std::size_t cell_of_unknown(std::size_t n) const noexcept { return cell_of_unknown_[n]; }
  [[nodiscard]] double diagonal(std::size_t n) const noexcept { return diag_[n]; }
  [[nodiscard]] const std::vector<FluidComponent>& components() const noexcept { return components_; }
  [[nodiscard]] std::uint32_t component_of(std::size_t n) const noexcept { return component_of_unknown_[n]; }
  [[nodiscard]] std::size_t memory_bytes() const noexcept;

  // y = A x (no allocation).
  void apply(std::span<const double> x, std::span<double> y) const;

  // b = -D u* over fluid unknowns (uses the stored face values as they are).
  void build_rhs(const MacGrid& mac, std::span<double> b) const;

  // u -= G phi on Coupled and Outlet faces only (prescribed faces are never touched).
  void correct_velocity(MacGrid& mac, std::span<const double> phi) const;

  [[nodiscard]] std::span<const double> inverse_diagonal() const noexcept { return inv_diag_; }

  // Read-only access to the discretization (used by the multigrid preconditioner; no new terms).
  // Bit s of coupled_faces(n) (s = 0..5: -x +x -y +y -z +z) is set when that face is Interior.
  [[nodiscard]] std::uint8_t coupled_faces(std::size_t n) const noexcept { return coupled_[n]; }
  // (1/dx^2, 1/dy^2, 1/dz^2): the off-diagonal weight of a coupled face along each axis.
  [[nodiscard]] std::array<double, 3> inverse_spacing_squared() const noexcept { return {idx2_, idy2_, idz2_}; }
  [[nodiscard]] std::array<std::size_t, 3> dims() const noexcept { return {nx_, ny_, nz_}; }
  [[nodiscard]] std::array<double, 3> spacing() const noexcept { return {dx_, dy_, dz_}; }
  [[nodiscard]] const BoundaryMap& boundary_map() const noexcept { return map_; }

 private:
  std::size_t nx_, ny_, nz_;
  double idx2_, idy2_, idz2_;
  double dx_, dy_, dz_;
  const BoundaryMap& map_;
  std::vector<std::int32_t> unknown_of_cell_;
  std::vector<std::uint32_t> cell_of_unknown_;
  std::vector<std::uint8_t> coupled_;  // bit 0..5: -x +x -y +y -z +z face is Coupled
  std::vector<double> diag_;      // A_cc (0 for an isolated single-cell pocket)
  std::vector<double> inv_diag_;  // Jacobi preconditioner (1 where A_cc = 0)
  std::vector<std::uint32_t> component_of_unknown_;
  std::vector<FluidComponent> components_;
};

struct SolveStats {
  std::size_t iterations = 0;      // total, including any after a fallback
  double rhs_norm = 0.0;           // ||b||_2
  double residual_norm = 0.0;      // true ||b - A phi||_2 after the solve
  double relative_residual = 0.0;  // residual_norm / rhs_norm (0 when b = 0)
  double max_residual = 0.0;       // true max_c |b - A phi|_c after the solve (= max |div| after, 1/s)
  bool converged = false;
  std::vector<double> history;     // relative (recursive) residual per iteration
  // Solver diagnostics (Milestone 7).
  PressureSolverKind requested = PressureSolverKind::Auto;
  PressureSolverKind selected = PressureSolverKind::JacobiPcg;  // Auto resolved for this grid
  PressureSolverKind used = PressureSolverKind::JacobiPcg;      // preconditioner in use at the end
  bool fallback = false;             // multigrid was requested but Jacobi-PCG finished the solve
  std::string fallback_reason;
  std::size_t fallback_iteration = 0;  // iterations done with multigrid before switching
  std::size_t multigrid_levels = 0;
  double setup_ms = 0.0;             // multigrid hierarchy built for this solve (0 when reused)
  double solve_ms = 0.0;             // PCG time, excluding hierarchy setup
};

// A fixed linear preconditioner z = M r, symmetric positive definite in the Euclidean inner product
// over pressure unknowns (the one used by solve_pressure).
class PressurePreconditioner {
 public:
  virtual ~PressurePreconditioner() = default;
  virtual void apply(std::span<const double> r, std::span<double> z) = 0;
};

// Reusable PCG storage: sized on first use, then reused without allocation (time stepping).
struct PcgWorkspace {
  std::vector<double> b, r, z, d, Ad, component_sum, component_abs, projected;
};

struct SolveOptions {
  bool initial_guess = false;   // start from the incoming phi (warm start) instead of 0
  bool record_history = true;   // per-iteration residuals (off in time loops: no allocation)
  // Mgpcg only: a prebuilt preconditioner (reused across solves). If null, solve_pressure builds a
  // multigrid hierarchy for this solve, unless `preconditioner_unavailable` gives the reason a
  // previous setup failed, in which case the solve falls back to Jacobi-PCG with that reason.
  PressurePreconditioner* preconditioner = nullptr;
  std::size_t preconditioner_levels = 0;
  std::string_view preconditioner_unavailable;
};

// Preconditioned CG on A phi = b (preconditioner per settings.solver). Components without an
// outlet (singular, constant nullspace) must have a compatible right-hand side: sum_c b_c = 0 to
// rounding; otherwise ProjectionError("incompatible_component"). Such components get a zero-mean
// gauge. Throws ProjectionError("projection_not_converged") if the tolerance is not met.
// The stopping test is always relative to ||b||, whatever the initial guess, and convergence is
// only reported from the true residual b - A phi.
//
// Fallback (Mgpcg): if the hierarchy cannot be built, or the preconditioner stops behaving as a
// positive definite map (r.z <= 0 or non-finite, or d.Ad <= 0), the solve continues with Jacobi-PCG
// from the current iterate, restarting CG from the true residual. The switch is reported in the
// statistics (fallback, fallback_reason, fallback_iteration); it is never silent.
SolveStats solve_pressure(const PressureSystem& system, std::span<const double> b, std::span<double> phi,
                          const ProjectionSettings& settings);
SolveStats solve_pressure(const PressureSystem& system, std::span<const double> b, std::span<double> phi,
                          const ProjectionSettings& settings, PcgWorkspace& workspace, const SolveOptions& options);

struct ProjectionResult {
  SolveStats solve;  // includes the solver diagnostics
  double density = 0.0;
  double dt = 0.0;
  double inlet_velocity = 0.0;
  DivergenceField divergence_before;
  DivergenceField divergence_after;  // computed independently with compute_divergence()
  std::optional<DivergenceStats> before_stats;
  std::optional<DivergenceStats> after_stats;
  BoundaryFluxes fluxes;
  std::size_t components = 0;
  std::size_t enclosed_components = 0;  // no outlet, compatible, zero-mean gauge
  std::size_t memory_bytes = 0;
  double setup_ms = 0.0;
  double solve_ms = 0.0;
  double total_ms = 0.0;
};

// Full projection of an intermediate field u* held in `mac`:
//   validate settings -> apply velocity BCs -> D u* -> b = -D u* -> solve A phi = b ->
//   u -= G phi (Coupled/Outlet faces) -> reapply prescribed BCs (no outlet copy) -> D u_new.
// Writes p = (rho/dt) phi into mac.p() (solid cells 0). Throws ProjectionError or BoundaryError.
ProjectionResult project_velocity(MacGrid& mac, const Grid3D& mask, const BoundaryMap& map,
                                  const BoundaryConditions& bc, const ProjectionSettings& settings);

// Cell-centered |U| = sqrt(ubar^2 + vbar^2 + wbar^2) from face-pair averages; solid cells 0 (invalid).
StaggeredArray velocity_magnitude(const MacGrid& mac, const Grid3D& mask);

class Multigrid;

// Repeated projections on one grid (time stepping): the pressure system, PCG workspace,
// divergence buffers, previous potential and (for Mgpcg) the multigrid hierarchy with its work
// buffers are built once and reused. Same operators and conventions as project_velocity.
// `mask` and `map` must outlive the projector. The hierarchy is built on the first Mgpcg
// projection, with that call's multigrid settings; a failed setup is remembered and reported
// as a fallback on every later Mgpcg solve (it is not retried).
//
// Warm start: the solve starts from the previous step's pressure, not its potential. Since
// phi = (dt/rho) p, the guess is phi_guess = (dt / dt_prev) phi_prev = (dt/rho) p_prev, which stays
// consistent when dt changes (e.g. a shortened frame-landing step). The convergence test is
// relative to ||b|| either way, so the warm start changes the iteration count, never the
// accepted accuracy. reset_warm_start() makes the next solve start cold (phi = 0).
class Projector {
 public:
  Projector(const Grid3D& mask, const BoundaryMap& map);
  ~Projector();
  Projector(const Projector&) = delete;
  Projector& operator=(const Projector&) = delete;

  struct Result {
    SolveStats solve;
    std::optional<DivergenceStats> before;  // of u* after velocity BCs
    std::optional<DivergenceStats> after;   // recomputed after the correction
    BoundaryFluxes fluxes;
    bool warm_started = false;  // the solve started from the rescaled previous potential
  };

  // apply_velocity_bcs -> b = -D u* -> PCG (warm start) -> u -= G phi -> prescribed BCs -> D u_new.
  // Writes p = (rho/dt) phi into mac.p(). Throws ProjectionError / BoundaryError.
  Result project(MacGrid& mac, const BoundaryConditions& bc, const ProjectionSettings& settings, double dt);

  // The next project() starts cold. Use after a projection whose potential is not a pressure of
  // the following steps (the initial projection of a prescribed field is an impulse).
  void reset_warm_start() noexcept { has_previous_ = false; }

  [[nodiscard]] const PressureSystem& system() const noexcept { return system_; }
  [[nodiscard]] const DivergenceField& divergence_after() const noexcept { return after_; }
  [[nodiscard]] std::size_t memory_bytes() const noexcept;
  // The cached hierarchy (null before the first Mgpcg projection or if its setup failed).
  [[nodiscard]] const Multigrid* multigrid() const noexcept { return multigrid_.get(); }
  [[nodiscard]] std::size_t multigrid_setups() const noexcept { return multigrid_setups_; }
  [[nodiscard]] const std::string& multigrid_error() const noexcept { return multigrid_error_; }

 private:
  const Grid3D& mask_;
  const BoundaryMap& map_;
  PressureSystem system_;
  PcgWorkspace workspace_;
  std::vector<double> b_, phi_;
  DivergenceField before_, after_;
  bool has_previous_ = false;
  double previous_dt_ = 0.0;  // dt of the solve that produced phi_
  std::unique_ptr<Multigrid> multigrid_;
  std::string multigrid_error_;
  std::size_t multigrid_setups_ = 0;
};

}  // namespace aeroflow
