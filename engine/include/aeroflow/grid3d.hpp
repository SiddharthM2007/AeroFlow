#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <vector>

#include "aeroflow/cell_type.hpp"
#include "aeroflow/vec3.hpp"

namespace aeroflow {

// Uniform Cartesian grid over the box [0, Lx] x [0, Ly] x [0, Lz].
//
// Conventions (approved for Milestone 1):
//   * Origin at the domain corner (0, 0, 0). Flow direction is +x, y is up.
//   * Spacing dx = Lx / Nx (likewise for y, z); cells need not be cubic.
//   * Cell (i, j, k) has center ((i + 0.5) dx, (j + 0.5) dy, (k + 0.5) dz).
//   * Linear index: idx = i + Nx * (j + Ny * k)  (i varies fastest).
//
// Cell classifications are stored contiguously in that linear order.
class Grid3D {
 public:
  // Upper bound on total cell count accepted by the constructor.
  static constexpr std::size_t kMaxCells = std::size_t{512} * 512 * 512;

  // Throws std::invalid_argument if any count is zero, the product overflows or
  // exceeds kMaxCells, or any length is not finite and strictly positive.
  Grid3D(std::size_t nx, std::size_t ny, std::size_t nz, double lx, double ly, double lz);

  [[nodiscard]] std::size_t nx() const noexcept { return nx_; }
  [[nodiscard]] std::size_t ny() const noexcept { return ny_; }
  [[nodiscard]] std::size_t nz() const noexcept { return nz_; }
  [[nodiscard]] double lx() const noexcept { return lx_; }
  [[nodiscard]] double ly() const noexcept { return ly_; }
  [[nodiscard]] double lz() const noexcept { return lz_; }
  [[nodiscard]] double dx() const noexcept { return dx_; }
  [[nodiscard]] double dy() const noexcept { return dy_; }
  [[nodiscard]] double dz() const noexcept { return dz_; }
  [[nodiscard]] double cell_volume() const noexcept { return dx_ * dy_ * dz_; }
  [[nodiscard]] std::size_t cell_count() const noexcept { return cells_.size(); }

  [[nodiscard]] bool in_bounds(std::size_t i, std::size_t j, std::size_t k) const noexcept {
    return i < nx_ && j < ny_ && k < nz_;
  }

  // 3D -> 1D. Unchecked; callers must ensure in_bounds(i, j, k).
  [[nodiscard]] std::size_t index(std::size_t i, std::size_t j, std::size_t k) const noexcept {
    return i + nx_ * (j + ny_ * k);
  }

  // 1D -> 3D. Unchecked; callers must ensure idx < cell_count().
  [[nodiscard]] std::array<std::size_t, 3> unravel(std::size_t idx) const noexcept {
    const std::size_t i = idx % nx_;
    const std::size_t rest = idx / nx_;
    return {i, rest % ny_, rest / ny_};
  }

  [[nodiscard]] double cell_center_x(std::size_t i) const noexcept;
  [[nodiscard]] double cell_center_y(std::size_t j) const noexcept;
  [[nodiscard]] double cell_center_z(std::size_t k) const noexcept;
  [[nodiscard]] Vec3 cell_center(std::size_t i, std::size_t j, std::size_t k) const noexcept;

  // Checked accessors; throw std::out_of_range for invalid indices.
  [[nodiscard]] CellType cell(std::size_t i, std::size_t j, std::size_t k) const;
  void set_cell(std::size_t i, std::size_t j, std::size_t k, CellType type);

  [[nodiscard]] std::span<const CellType> cells() const noexcept { return cells_; }
  [[nodiscard]] std::span<CellType> cells() noexcept { return cells_; }

  [[nodiscard]] std::size_t count(CellType type) const noexcept;

  void fill(CellType type) noexcept;

 private:
  std::size_t checked_index(std::size_t i, std::size_t j, std::size_t k) const;

  std::size_t nx_;
  std::size_t ny_;
  std::size_t nz_;
  double lx_;
  double ly_;
  double lz_;
  double dx_;
  double dy_;
  double dz_;
  std::vector<CellType> cells_;
};

}  // namespace aeroflow
