#pragma once

#include <cstddef>

namespace aeroflow {

// Multigrid V-cycle parameters (see multigrid.hpp). Kept separate so the pressure solver settings
// can carry them without depending on the hierarchy itself.
// Grid transfers: piecewise-constant (parent only, the default) or trilinear prolongation; the
// restriction is the transpose in both cases. Measured in Milestone 7 phase 4, piecewise-constant
// transfers gave the fastest solves on every benchmark (box, Valve Body STL, anisotropic; 65k-2.1M
// cells): fewer PCG iterations at a cheaper cycle. With rediscretized coarse operators they
// over-correct by about 2 (P^T K P is twice K_coarse), which helps; B stays symmetric positive
// definite either way.
enum class MultigridTransfer { Trilinear, Constant };

struct MultigridSettings {
  std::size_t pre_sweeps = 2;          // red-black sweeps before the coarse correction
  std::size_t post_sweeps = 2;         // reversed sweeps after it (must equal pre_sweeps)
  std::size_t coarse_sweeps = 16;      // coarsest level: this many forward, then reversed sweeps
  std::size_t coarsest_unknowns = 512; // stop coarsening at or below this many unknowns
  std::size_t max_levels = 16;
  double semi_coarsening_ratio = 1.5;  // coarsen axis a only if width_a <= ratio * min width
  MultigridTransfer transfer = MultigridTransfer::Constant;
};

// Throws std::invalid_argument for zero sweeps, unequal pre/post sweeps, max_levels == 0, or a
// ratio < 1.
void validate_multigrid_settings(const MultigridSettings& s);

}  // namespace aeroflow
