#include "aeroflow/mac_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace aeroflow {

namespace {

constexpr std::size_t kSizeMax = std::numeric_limits<std::size_t>::max();

// a * b with overflow detection.
std::size_t mul(std::size_t a, std::size_t b) {
  if (a != 0 && b > kSizeMax / a) throw std::invalid_argument("MAC grid size overflows size_t");
  return a * b;
}

std::size_t add(std::size_t a, std::size_t b) {
  if (b > kSizeMax - a) throw std::invalid_argument("MAC grid size overflows size_t");
  return a + b;
}

std::size_t inc(std::size_t n) { return add(n, 1); }

std::size_t volume(std::size_t a, std::size_t b, std::size_t c) { return mul(mul(a, b), c); }

void require_count(std::size_t n, const char* name) {
  if (n == 0) throw std::invalid_argument(std::string(name) + " must be at least 1");
}

void require_length(double l, const char* name) {
  if (!std::isfinite(l) || l <= 0.0) throw std::invalid_argument(std::string(name) + " must be a finite positive number");
}

std::string ijk(std::size_t i, std::size_t j, std::size_t k) {
  return "(" + std::to_string(i) + ", " + std::to_string(j) + ", " + std::to_string(k) + ")";
}

FaceType classify(const CellType* low, const CellType* high) {
  // low/high: the cells on either side of the face; nullptr on the tunnel wall.
  if (!low || !high) {
    const CellType only = low ? *low : *high;
    return only == CellType::Fluid ? FaceType::DomainBoundary : FaceType::SolidInterior;
  }
  const bool lf = *low == CellType::Fluid;
  const bool hf = *high == CellType::Fluid;
  if (lf && hf) return FaceType::Fluid;
  if (lf || hf) return FaceType::SolidBoundary;
  return FaceType::SolidInterior;
}

}  // namespace

// --- StaggeredArray -------------------------------------------------------------------------

StaggeredArray::StaggeredArray(std::size_t n0, std::size_t n1, std::size_t n2, double value)
    : n0_(n0), n1_(n1), n2_(n2) {
  if (n0 == 0 || n1 == 0 || n2 == 0) throw std::invalid_argument("array extents must be at least 1");
  data_.assign(volume(n0, n1, n2), value);
}

std::size_t StaggeredArray::checked_index(std::size_t i, std::size_t j, std::size_t k) const {
  if (!in_bounds(i, j, k)) {
    throw std::out_of_range("index " + ijk(i, j, k) + " is outside array " + std::to_string(n0_) + "x" +
                            std::to_string(n1_) + "x" + std::to_string(n2_));
  }
  return index(i, j, k);
}

double& StaggeredArray::at(std::size_t i, std::size_t j, std::size_t k) { return data_[checked_index(i, j, k)]; }
double StaggeredArray::at(std::size_t i, std::size_t j, std::size_t k) const { return data_[checked_index(i, j, k)]; }

void StaggeredArray::fill(double value) noexcept { std::fill(data_.begin(), data_.end(), value); }

// --- FaceTypes ------------------------------------------------------------------------------

namespace {

FaceType face_at(const std::vector<FaceType>& faces, std::size_t n0, std::size_t n1, std::size_t n2,
                 std::size_t i, std::size_t j, std::size_t k, const char* name) {
  if (i >= n0 || j >= n1 || k >= n2) {
    throw std::out_of_range(std::string(name) + " face " + ijk(i, j, k) + " is out of range");
  }
  return faces[i + n0 * (j + n1 * k)];
}

}  // namespace

FaceType FaceTypes::u_at(std::size_t i, std::size_t j, std::size_t k) const {
  return face_at(u, nx + 1, ny, nz, i, j, k, "u");
}
FaceType FaceTypes::v_at(std::size_t i, std::size_t j, std::size_t k) const {
  return face_at(v, nx, ny + 1, nz, i, j, k, "v");
}
FaceType FaceTypes::w_at(std::size_t i, std::size_t j, std::size_t k) const {
  return face_at(w, nx, ny, nz + 1, i, j, k, "w");
}

std::size_t FaceTypes::count(FaceType type) const noexcept {
  return static_cast<std::size_t>(std::count(u.begin(), u.end(), type) + std::count(v.begin(), v.end(), type) +
                                  std::count(w.begin(), w.end(), type));
}

FaceTypes classify_faces(const Grid3D& mask) {
  const std::size_t nx = mask.nx(), ny = mask.ny(), nz = mask.nz();
  const auto cells = mask.cells();
  auto cell = [&](std::size_t i, std::size_t j, std::size_t k) { return &cells[mask.index(i, j, k)]; };

  FaceTypes f;
  f.nx = nx;
  f.ny = ny;
  f.nz = nz;
  f.u.resize(volume(inc(nx), ny, nz));
  f.v.resize(volume(nx, inc(ny), nz));
  f.w.resize(volume(nx, ny, inc(nz)));

  for (std::size_t k = 0; k < nz; ++k)
    for (std::size_t j = 0; j < ny; ++j)
      for (std::size_t i = 0; i <= nx; ++i)
        f.u[i + (nx + 1) * (j + ny * k)] = classify(i > 0 ? cell(i - 1, j, k) : nullptr, i < nx ? cell(i, j, k) : nullptr);

  for (std::size_t k = 0; k < nz; ++k)
    for (std::size_t j = 0; j <= ny; ++j)
      for (std::size_t i = 0; i < nx; ++i)
        f.v[i + nx * (j + (ny + 1) * k)] = classify(j > 0 ? cell(i, j - 1, k) : nullptr, j < ny ? cell(i, j, k) : nullptr);

  for (std::size_t k = 0; k <= nz; ++k)
    for (std::size_t j = 0; j < ny; ++j)
      for (std::size_t i = 0; i < nx; ++i)
        f.w[i + nx * (j + ny * k)] = classify(k > 0 ? cell(i, j, k - 1) : nullptr, k < nz ? cell(i, j, k) : nullptr);

  return f;
}

// --- MacGrid --------------------------------------------------------------------------------

std::size_t MacGrid::required_bytes(std::size_t nx, std::size_t ny, std::size_t nz) {
  const std::size_t doubles = add(add(add(volume(nx, ny, nz), volume(inc(nx), ny, nz)), volume(nx, inc(ny), nz)),
                                  volume(nx, ny, inc(nz)));
  return mul(doubles, sizeof(double));
}

MacGrid::MacGrid(std::size_t nx, std::size_t ny, std::size_t nz, double lx, double ly, double lz,
                 std::size_t max_bytes)
    : nx_(nx), ny_(ny), nz_(nz), lx_(lx), ly_(ly), lz_(lz), dx_(0), dy_(0), dz_(0) {
  require_count(nx, "nx");
  require_count(ny, "ny");
  require_count(nz, "nz");
  require_length(lx, "lx");
  require_length(ly, "ly");
  require_length(lz, "lz");
  const std::size_t cells = volume(nx, ny, nz);
  if (cells > Grid3D::kMaxCells) {
    throw std::invalid_argument("MAC grid has " + std::to_string(cells) + " cells; maximum is " +
                                std::to_string(Grid3D::kMaxCells));
  }
  const std::size_t bytes = required_bytes(nx, ny, nz);
  if (bytes > max_bytes) {
    throw std::invalid_argument("MAC grid fields need " + std::to_string(bytes) + " bytes; limit is " +
                                std::to_string(max_bytes));
  }
  // Same expressions as Grid3D, so spacings (and therefore positions) are bitwise identical.
  dx_ = lx_ / static_cast<double>(nx_);
  dy_ = ly_ / static_cast<double>(ny_);
  dz_ = lz_ / static_cast<double>(nz_);
  p_ = StaggeredArray(nx, ny, nz);
  u_ = StaggeredArray(nx + 1, ny, nz);
  v_ = StaggeredArray(nx, ny + 1, nz);
  w_ = StaggeredArray(nx, ny, nz + 1);
}

MacGrid::MacGrid(const Grid3D& grid, std::size_t max_bytes)
    : MacGrid(grid.nx(), grid.ny(), grid.nz(), grid.lx(), grid.ly(), grid.lz(), max_bytes) {}

bool MacGrid::matches(const Grid3D& grid) const noexcept {
  return grid.nx() == nx_ && grid.ny() == ny_ && grid.nz() == nz_ && grid.lx() == lx_ && grid.ly() == ly_ &&
         grid.lz() == lz_;
}

Vec3 MacGrid::p_position(std::size_t i, std::size_t j, std::size_t k) const noexcept {
  return {(static_cast<double>(i) + 0.5) * dx_, (static_cast<double>(j) + 0.5) * dy_,
          (static_cast<double>(k) + 0.5) * dz_};
}
Vec3 MacGrid::u_position(std::size_t i, std::size_t j, std::size_t k) const noexcept {
  return {static_cast<double>(i) * dx_, (static_cast<double>(j) + 0.5) * dy_, (static_cast<double>(k) + 0.5) * dz_};
}
Vec3 MacGrid::v_position(std::size_t i, std::size_t j, std::size_t k) const noexcept {
  return {(static_cast<double>(i) + 0.5) * dx_, static_cast<double>(j) * dy_, (static_cast<double>(k) + 0.5) * dz_};
}
Vec3 MacGrid::w_position(std::size_t i, std::size_t j, std::size_t k) const noexcept {
  return {(static_cast<double>(i) + 0.5) * dx_, (static_cast<double>(j) + 0.5) * dy_, static_cast<double>(k) * dz_};
}

void MacGrid::set_zero() noexcept {
  p_.fill(0.0);
  u_.fill(0.0);
  v_.fill(0.0);
  w_.fill(0.0);
}

void MacGrid::set_uniform_velocity(const Vec3& velocity, FaceSelection selection, const FaceTypes* faces) {
  if (selection == FaceSelection::All) {
    u_.fill(velocity.x);
    v_.fill(velocity.y);
    w_.fill(velocity.z);
    return;
  }
  if (!faces) throw std::invalid_argument("FaceSelection::ActiveOnly requires face types");
  if (faces->nx != nx_ || faces->ny != ny_ || faces->nz != nz_ || faces->u.size() != u_.size() ||
      faces->v.size() != v_.size() || faces->w.size() != w_.size()) {
    throw std::invalid_argument("face types do not match the MAC grid dimensions");
  }
  auto assign = [](StaggeredArray& field, const std::vector<FaceType>& types, double value) {
    auto data = field.data();
    for (std::size_t n = 0; n < data.size(); ++n) data[n] = is_active(types[n]) ? value : 0.0;
  };
  assign(u_, faces->u, velocity.x);
  assign(v_, faces->v, velocity.y);
  assign(w_, faces->w, velocity.z);
}

}  // namespace aeroflow
