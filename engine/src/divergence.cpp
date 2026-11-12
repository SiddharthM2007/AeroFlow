#include "aeroflow/divergence.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aeroflow {

DivergenceField compute_divergence(const MacGrid& mac, const Grid3D& mask) {
  if (!mac.matches(mask)) throw std::invalid_argument("solid mask does not match the MAC grid");

  const std::size_t nx = mac.nx(), ny = mac.ny(), nz = mac.nz();
  const double dx = mac.dx(), dy = mac.dy(), dz = mac.dz();
  const auto& u = mac.u();
  const auto& v = mac.v();
  const auto& w = mac.w();
  const auto cells = mask.cells();

  DivergenceField out{StaggeredArray(nx, ny, nz, 0.0), 0};
  for (std::size_t k = 0; k < nz; ++k) {
    for (std::size_t j = 0; j < ny; ++j) {
      for (std::size_t i = 0; i < nx; ++i) {
        if (cells[mask.index(i, j, k)] != CellType::Fluid) continue;
        out.values(i, j, k) = (u(i + 1, j, k) - u(i, j, k)) / dx + (v(i, j + 1, k) - v(i, j, k)) / dy +
                              (w(i, j, k + 1) - w(i, j, k)) / dz;
        ++out.fluid_cells;
      }
    }
  }
  return out;
}

std::size_t count_fluid_cells(const Grid3D& mask) noexcept { return mask.count(CellType::Fluid); }

std::optional<DivergenceStats> divergence_stats(const DivergenceField& div, const Grid3D& mask) {
  if (div.values.n0() != mask.nx() || div.values.n1() != mask.ny() || div.values.n2() != mask.nz()) {
    throw std::invalid_argument("divergence field does not match the solid mask");
  }
  DivergenceStats s;
  double sum_abs = 0.0;
  double sum_sq = 0.0;
  const auto cells = mask.cells();
  const auto values = div.values.data();
  for (std::size_t n = 0; n < values.size(); ++n) {
    if (cells[n] != CellType::Fluid) continue;  // same x-fastest layout as the mask
    const double a = std::abs(values[n]);
    s.max_abs = std::max(s.max_abs, a);
    sum_abs += a;
    sum_sq += a * a;
    ++s.fluid_cells;
  }
  if (s.fluid_cells == 0) return std::nullopt;
  const double count = static_cast<double>(s.fluid_cells);
  s.mean_abs = sum_abs / count;
  s.rms = std::sqrt(sum_sq / count);
  return s;
}

}  // namespace aeroflow
