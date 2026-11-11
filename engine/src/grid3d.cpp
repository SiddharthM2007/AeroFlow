#include "aeroflow/grid3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace aeroflow {

namespace {

void require_positive_length(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be a finite positive number");
  }
}

void require_nonzero_count(std::size_t value, const char* name) {
  if (value == 0) {
    throw std::invalid_argument(std::string(name) + " must be at least 1");
  }
}

std::size_t checked_cell_count(std::size_t nx, std::size_t ny, std::size_t nz) {
  constexpr std::size_t kMax = std::numeric_limits<std::size_t>::max();
  if (nx > kMax / ny || nx * ny > kMax / nz) {
    throw std::invalid_argument("grid cell count overflows size_t");
  }
  const std::size_t total = nx * ny * nz;
  if (total > Grid3D::kMaxCells) {
    throw std::invalid_argument("grid has " + std::to_string(total) + " cells; maximum is " +
                                std::to_string(Grid3D::kMaxCells));
  }
  return total;
}

}  // namespace

Grid3D::Grid3D(std::size_t nx, std::size_t ny, std::size_t nz, double lx, double ly, double lz)
    : nx_(nx), ny_(ny), nz_(nz), lx_(lx), ly_(ly), lz_(lz), dx_(0.0), dy_(0.0), dz_(0.0) {
  require_nonzero_count(nx, "nx");
  require_nonzero_count(ny, "ny");
  require_nonzero_count(nz, "nz");
  require_positive_length(lx, "lx");
  require_positive_length(ly, "ly");
  require_positive_length(lz, "lz");
  const std::size_t total = checked_cell_count(nx, ny, nz);

  dx_ = lx_ / static_cast<double>(nx_);
  dy_ = ly_ / static_cast<double>(ny_);
  dz_ = lz_ / static_cast<double>(nz_);
  cells_.assign(total, CellType::Fluid);
}

double Grid3D::cell_center_x(std::size_t i) const noexcept {
  return (static_cast<double>(i) + 0.5) * dx_;
}

double Grid3D::cell_center_y(std::size_t j) const noexcept {
  return (static_cast<double>(j) + 0.5) * dy_;
}

double Grid3D::cell_center_z(std::size_t k) const noexcept {
  return (static_cast<double>(k) + 0.5) * dz_;
}

Vec3 Grid3D::cell_center(std::size_t i, std::size_t j, std::size_t k) const noexcept {
  return {cell_center_x(i), cell_center_y(j), cell_center_z(k)};
}

std::size_t Grid3D::checked_index(std::size_t i, std::size_t j, std::size_t k) const {
  if (!in_bounds(i, j, k)) {
    throw std::out_of_range("cell (" + std::to_string(i) + ", " + std::to_string(j) + ", " +
                            std::to_string(k) + ") is outside grid " + std::to_string(nx_) + "x" +
                            std::to_string(ny_) + "x" + std::to_string(nz_));
  }
  return index(i, j, k);
}

CellType Grid3D::cell(std::size_t i, std::size_t j, std::size_t k) const {
  return cells_[checked_index(i, j, k)];
}

void Grid3D::set_cell(std::size_t i, std::size_t j, std::size_t k, CellType type) {
  cells_[checked_index(i, j, k)] = type;
}

std::size_t Grid3D::count(CellType type) const noexcept {
  return static_cast<std::size_t>(std::count(cells_.begin(), cells_.end(), type));
}

void Grid3D::fill(CellType type) noexcept {
  std::fill(cells_.begin(), cells_.end(), type);
}

}  // namespace aeroflow
