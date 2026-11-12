#pragma once

#include <cstddef>
#include <optional>

#include "aeroflow/grid3d.hpp"
#include "aeroflow/mac_grid.hpp"

namespace aeroflow {

// Cell-centered discrete divergence of the MAC velocity field (Nx x Ny x Nz, x-fastest).
//
// For a fluid cell (i,j,k):
//   div = (u(i+1,j,k) - u(i,j,k)) / dx + (v(i,j+1,k) - v(i,j,k)) / dy + (w(i,j,k+1) - w(i,j,k)) / dz
// i.e. the net outflow through the cell's six faces divided by its volume (finite-volume form).
//
// Solid cells hold 0.0, which carries no physical meaning: a value is valid only where the
// mask cell is Fluid. Face values are used exactly as stored; blocked faces are not modified.
// This only measures divergence; nothing here corrects it.
struct DivergenceField {
  StaggeredArray values;
  std::size_t fluid_cells = 0;  // number of valid entries
};

// Throws std::invalid_argument if `mask` does not describe the same grid as `mac`.
DivergenceField compute_divergence(const MacGrid& mac, const Grid3D& mask);

std::size_t count_fluid_cells(const Grid3D& mask) noexcept;

struct DivergenceStats {
  std::size_t fluid_cells = 0;
  double max_abs = 0.0;
  double mean_abs = 0.0;
  double rms = 0.0;
};

// Statistics over fluid cells only. Returns std::nullopt when there are no fluid cells.
// Throws std::invalid_argument if the field and mask dimensions differ.
std::optional<DivergenceStats> divergence_stats(const DivergenceField& div, const Grid3D& mask);

}  // namespace aeroflow
