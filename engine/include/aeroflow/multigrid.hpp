#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "aeroflow/boundary.hpp"
#include "aeroflow/grid3d.hpp"
#include "aeroflow/multigrid_settings.hpp"
#include "aeroflow/pressure.hpp"

namespace aeroflow {

// Geometric multigrid V-cycle for the pressure operator A of PressureSystem (Milestone 7).
// It is meant as a preconditioner z = B r for conjugate gradients: B is a fixed linear operator,
// symmetric and positive definite in the Euclidean inner product over fluid unknowns (the inner
// product used by solve_pressure), whatever the geometry. It never replaces A: the fine level IS
// the PressureSystem operator, and convergence is always judged on A.
//
// Conductance form. A is the volume-normalized operator A = K_0 / V_0, where V_0 = dx dy dz and
//   (K phi)_c = sum_faces T_f (phi_c - phi_nb) + sum_outlet T_o phi_c,
//   T_f = (open face area) / (centre-to-centre distance),  T_o = (open outlet area) / (half width).
// Coarse operators K_l are rediscretized in this form from the open face areas of the finer level
// (face-opening aggregation). K_l is symmetric by construction (one T per face) and positive
// semidefinite (weighted graph Laplacian plus a non-negative outlet diagonal). Unequal coarse
// cell volumes (partial cells on odd grids) enter only through the true centre distances and
// widths; no volume weighting is needed, so the Euclidean inner product is the natural one on
// every level and restriction is exactly R = P^T.
//
// Hierarchy. Cell-centred 2:1 coarsening per axis; an odd count n becomes ceil(n/2) with a
// one-child (half-width) last cell. Semi-coarsening: an axis is coarsened only while its width
// is <= ratio x the smallest width (default 1.5), so point smoothing stays effective on
// anisotropic grids. A coarse cell is fluid if any child is fluid.
//
// Transfers. Piecewise-constant by default (see MultigridSettings), or trilinear prolongation using
// true cell centres (weights 3/4, 1/4 for full cells;
// other values next to partial cells). Along an axis, the neighbour is used only if the coarse
// face to it is open; a stencil cell that is solid or lies in another coarse fluid component
// gives its weight to the parent, so every row of P sums to 1 (constants are reproduced).
// Restriction is the literal transpose, computed from the same stencil.
//
// Cycle. Symmetric V(pre, post) with red-black Gauss-Seidel: pre-smoothing sweeps red then
// black, post-smoothing black then red (the adjoint ordering). The coarsest level is smoothed
// with a fixed number of forward then reversed sweeps (a fixed linear symmetric map, no tolerance
// test). Level 0's residual is restricted with the factor V_0 (K_0 = V_0 A). With R = P^T,
// adjoint smoothers and SPD/PSD coarse maps, B = Mbar + (I - A M)^T P B_c P^T (I - A M) is
// symmetric, and positive definite because Mbar (symmetric Gauss-Seidel with a positive
// diagonal) is.
//
// Special unknowns. A fluid cell with no open face and no outlet (isolated pocket, zero row) is
// smoothed with a unit diagonal (as the Jacobi preconditioner does), keeping B positive definite.
// Enclosed Neumann regions (no outlet) are singular on every level; Gauss-Seidel with a positive
// diagonal is still well defined and B stays positive definite. Coarse levels may merge fluid
// regions that are separated by walls thinner than a coarse cell; this only changes the quality
// of B, never its symmetry or definiteness.


struct MultigridLevelInfo {
  std::array<std::size_t, 3> dims{};
  std::array<bool, 3> coarsened{};  // relative to the finer level (all false on level 0)
  std::size_t unknowns = 0;
  std::size_t components = 0;
  std::size_t enclosed_components = 0;  // no outlet (singular, constant nullspace)
  std::size_t isolated_cells = 0;       // zero rows (smoothed with a unit diagonal)
  std::size_t memory_bytes = 0;
};

// Accumulated V-cycle stage times (milliseconds), when profiling is enabled.
struct MultigridProfile {
  std::size_t cycles = 0;
  double fine_smooth = 0, fine_residual = 0, fine_restrict = 0, fine_prolong = 0;  // level 0
  double coarse_levels = 0;  // everything on levels >= 1 (including the coarsest solve)
};

class Multigrid final : public PressurePreconditioner {
 public:
  // `system` and `map` must outlive the hierarchy and describe the same grid as `mask`.
  Multigrid(const Grid3D& mask, const BoundaryMap& map, const PressureSystem& system,
            MultigridSettings settings = {});
  // Same, taking the grid and boundary map from the pressure system.
  explicit Multigrid(const PressureSystem& system, MultigridSettings settings = {});

  // z = B r over the fine unknowns: one symmetric V-cycle from a zero initial guess.
  // Fixed work, linear in r, no allocation.
  void apply(std::span<const double> r, std::span<double> z) override;

  [[nodiscard]] std::size_t levels() const noexcept { return levels_.size(); }
  [[nodiscard]] const MultigridLevelInfo& info(std::size_t level) const { return info_.at(level); }
  [[nodiscard]] std::size_t memory_bytes() const noexcept;
  [[nodiscard]] const MultigridSettings& settings() const noexcept { return settings_; }
  [[nodiscard]] double setup_ms() const noexcept { return setup_ms_; }
  // Stage timing for benchmarks (off by default; a few clock reads per cycle when on).
  void set_profiling(bool on) noexcept { profiling_ = on; }
  [[nodiscard]] const MultigridProfile& profile() const noexcept { return profile_; }
  void reset_profile() noexcept { profile_ = {}; }

  // --- inspection (tests and benchmarks) ---
  [[nodiscard]] std::size_t unknowns(std::size_t level) const {
    return level == 0 ? system_.unknowns() : levels_.at(level).cell_of_unknown.size();
  }
  // y = A x on level 0 (computed by the multigrid's own stencil code), y = K_l x on coarse levels.
  void apply_operator(std::size_t level, std::span<const double> x, std::span<double> y) const;
  // Fine level `level` <-> coarse level `level + 1`: y = P x (prolongation) and y = P^T x.
  void prolong(std::size_t level, std::span<const double> coarse, std::span<double> fine) const;
  void restrict_transpose(std::size_t level, std::span<const double> fine, std::span<double> coarse) const;
  // Unoptimized per-unknown stencil evaluation of P (the Phase 2 code), kept to test the cached path.
  void prolong_reference(std::size_t level, std::span<const double> coarse, std::span<double> fine) const;
  [[nodiscard]] std::span<const double> widths(std::size_t level, int axis) const { return levels_.at(level).width[static_cast<std::size_t>(axis)]; }
  [[nodiscard]] std::span<const double> centers(std::size_t level, int axis) const { return levels_.at(level).center[static_cast<std::size_t>(axis)]; }
  // Conductance of the face below cell index i along `axis` on a coarse level (face arrays have
  // n_axis + 1 entries per line; the +x boundary face holds the outlet conductance).
  [[nodiscard]] double conductance(std::size_t level, int axis, std::size_t i, std::size_t j, std::size_t k) const;
  [[nodiscard]] std::int32_t unknown_of_cell(std::size_t level, std::size_t i, std::size_t j, std::size_t k) const;
  [[nodiscard]] std::uint32_t component_of(std::size_t level, std::size_t unknown) const {
    return levels_.at(level).component[unknown];
  }

 private:
  struct Level {
    std::array<std::size_t, 3> n{};
    std::array<bool, 3> coarsened{};
    std::array<std::vector<double>, 3> width, center;
    std::vector<std::int32_t> unknown_of_cell;
    std::vector<std::uint32_t> cell_of_unknown;
    std::array<std::vector<double>, 3> cond;  // coarse levels: face conductances; empty on level 0
    std::array<std::vector<double>, 3> area;  // coarse levels: open face areas (to build the next level)
    std::vector<double> diag;                 // coarse levels; level 0 uses PressureSystem::diagonal
    std::vector<std::uint32_t> red, black;    // unknowns by parity of i + j + k
    std::vector<std::uint32_t> component;
    std::vector<std::uint8_t> component_has_outlet;
    // Transfer to the next coarser level, per axis and per index along the axis.
    std::array<std::vector<std::uint32_t>, 3> parent;
    std::array<std::vector<std::int8_t>, 3> step;  // -1, 0, +1: neighbour used for interpolation
    std::array<std::vector<double>, 3> weight;     // neighbour weight in [0, 1/2]
    // Per fine unknown: bits 0-7 = stencil cell (combo of shifted axes) is fluid and in the parent's
    // coarse region; bits 8-10 = interpolation along x/y/z is allowed (open coarse face).
    std::vector<std::uint16_t> transfer_mask;
    std::vector<double> x, f, r;                   // work vectors (level 0: only r)
  };

  struct Stencil {
    std::array<std::uint32_t, 8> unknown{};
    std::array<double, 8> weight{};
    int size = 0;
  };

  void build(std::array<std::size_t, 3> dims, std::array<double, 3> spacing);
  void build_level0(std::array<std::size_t, 3> dims, std::array<double, 3> spacing);
  void build_coarse(std::size_t fine);
  void build_transfer(std::size_t fine);
  void build_transfer_masks(std::size_t fine);
  template <typename Emit>
  void for_each_transfer(std::size_t fine, Emit&& emit) const;
  void finish_level(std::size_t level);
  [[nodiscard]] Stencil stencil(std::size_t fine, std::size_t u) const;
  void prolong_add(std::size_t level, std::span<const double> coarse, std::span<double> fine) const;
  [[nodiscard]] double off_diagonal_sum(std::size_t level, std::uint32_t u, std::span<const double> x) const;
  [[nodiscard]] double diagonal(std::size_t level, std::uint32_t u) const;
  void sweep(std::size_t level, const std::vector<std::uint32_t>& color, std::span<const double> f, std::span<double> x) const;
  void smooth(std::size_t level, std::size_t sweeps, bool reversed, std::span<const double> f, std::span<double> x) const;
  void residual(std::size_t level, std::span<const double> f, std::span<const double> x, std::span<double> r) const;

  const BoundaryMap& map_;
  const PressureSystem& system_;
  MultigridSettings settings_;
  std::vector<Level> levels_;
  std::vector<MultigridLevelInfo> info_;
  double v0_ = 1.0;  // fine cell volume: K_0 = v0_ * A
  double setup_ms_ = 0.0;
  bool profiling_ = false;
  MultigridProfile profile_;
};

}  // namespace aeroflow
