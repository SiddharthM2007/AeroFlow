#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "aeroflow/grid3d.hpp"
#include "aeroflow/vec3.hpp"

namespace aeroflow {

// Contiguous 3D array of doubles with x-fastest indexing: idx = i + n0 * (j + n1 * k).
class StaggeredArray {
 public:
  StaggeredArray() = default;
  // Throws std::invalid_argument if any extent is zero or the element count overflows.
  StaggeredArray(std::size_t n0, std::size_t n1, std::size_t n2, double value = 0.0);

  [[nodiscard]] std::size_t n0() const noexcept { return n0_; }
  [[nodiscard]] std::size_t n1() const noexcept { return n1_; }
  [[nodiscard]] std::size_t n2() const noexcept { return n2_; }
  [[nodiscard]] std::size_t size() const noexcept { return data_.size(); }

  [[nodiscard]] bool in_bounds(std::size_t i, std::size_t j, std::size_t k) const noexcept {
    return i < n0_ && j < n1_ && k < n2_;
  }
  // Unchecked; callers must ensure in_bounds(i, j, k).
  [[nodiscard]] std::size_t index(std::size_t i, std::size_t j, std::size_t k) const noexcept {
    return i + n0_ * (j + n1_ * k);
  }
  [[nodiscard]] double& operator()(std::size_t i, std::size_t j, std::size_t k) noexcept {
    return data_[index(i, j, k)];
  }
  [[nodiscard]] double operator()(std::size_t i, std::size_t j, std::size_t k) const noexcept {
    return data_[index(i, j, k)];
  }
  // Checked; throw std::out_of_range.
  [[nodiscard]] double& at(std::size_t i, std::size_t j, std::size_t k);
  [[nodiscard]] double at(std::size_t i, std::size_t j, std::size_t k) const;

  [[nodiscard]] std::span<double> data() noexcept { return data_; }
  [[nodiscard]] std::span<const double> data() const noexcept { return data_; }
  void fill(double value) noexcept;

 private:
  std::size_t checked_index(std::size_t i, std::size_t j, std::size_t k) const;

  std::size_t n0_ = 0;
  std::size_t n1_ = 0;
  std::size_t n2_ = 0;
  std::vector<double> data_;
};

// Classification of a velocity face from the solid/fluid cells on either side of it.
// The underlying values are stable (one byte per face) for reuse by later modules.
enum class FaceType : std::uint8_t {
  Fluid = 0,           // interior face, fluid | fluid: active
  SolidBoundary = 1,   // interior face, fluid | solid: obstacle wall
  SolidInterior = 2,   // solid | solid, or a tunnel-wall face whose only cell is solid: inactive
  DomainBoundary = 3,  // tunnel-wall face whose only cell is fluid
};

// Fluid and DomainBoundary faces carry flow; SolidBoundary and SolidInterior faces are blocked.
[[nodiscard]] constexpr bool is_active(FaceType t) noexcept {
  return t == FaceType::Fluid || t == FaceType::DomainBoundary;
}

// Face types laid out exactly like MacGrid::u(), v(), w() (same extents and indexing).
struct FaceTypes {
  std::size_t nx = 0;
  std::size_t ny = 0;
  std::size_t nz = 0;
  std::vector<FaceType> u;  // (nx+1) x ny x nz
  std::vector<FaceType> v;  // nx x (ny+1) x nz
  std::vector<FaceType> w;  // nx x ny x (nz+1)

  // Checked accessors (std::out_of_range).
  [[nodiscard]] FaceType u_at(std::size_t i, std::size_t j, std::size_t k) const;
  [[nodiscard]] FaceType v_at(std::size_t i, std::size_t j, std::size_t k) const;
  [[nodiscard]] FaceType w_at(std::size_t i, std::size_t j, std::size_t k) const;

  [[nodiscard]] std::size_t count(FaceType type) const noexcept;
};

// Classifies every u, v and w face of the grid from its solid/fluid cell mask.
//   interior face:  fluid|fluid -> Fluid, fluid|solid -> SolidBoundary, solid|solid -> SolidInterior
//   tunnel wall:    fluid       -> DomainBoundary, solid -> SolidInterior
// Obstacle walls (SolidBoundary) are never conflated with tunnel walls (DomainBoundary).
FaceTypes classify_faces(const Grid3D& mask);

enum class FaceSelection {
  All,         // every face, including blocked ones (testing only; no physical meaning there)
  ActiveOnly,  // Fluid and DomainBoundary faces; blocked faces are set to 0.0
};

// Staggered Marker-and-Cell grid over the same domain and cells as Grid3D.
//
// Field placement (origin at the domain corner, flow +x, y up):
//   p(i,j,k) : Nx   x Ny   x Nz      at ((i+.5)dx, (j+.5)dy, (k+.5)dz)   cell centers
//   u(i,j,k) : Nx+1 x Ny   x Nz      at ( i    dx, (j+.5)dy, (k+.5)dz)   x-faces
//   v(i,j,k) : Nx   x Ny+1 x Nz      at ((i+.5)dx,  j    dy, (k+.5)dz)   y-faces
//   w(i,j,k) : Nx   x Ny   x Nz+1    at ((i+.5)dx, (j+.5)dy,  k    dz)   z-faces
//
// Cell (i,j,k) is bounded by u(i,j,k) / u(i+1,j,k), v(i,j,k) / v(i,j+1,k), w(i,j,k) / w(i,j,k+1).
// Every field is a separate contiguous double array with x-fastest indexing, and dx = Lx/Nx
// (etc.) is computed exactly as in Grid3D, so positions match the voxelizer's cell centers.
class MacGrid {
 public:
  // Allocation guard: p, u, v, w together may not exceed this many bytes by default (~256^3 fits).
  static constexpr std::size_t kDefaultMaxBytes = std::size_t{2} << 30;  // 2 GiB

  // Throws std::invalid_argument for zero counts, non-finite or non-positive lengths, more than
  // Grid3D::kMaxCells cells, size overflow, or a field allocation larger than max_bytes.
  MacGrid(std::size_t nx, std::size_t ny, std::size_t nz, double lx, double ly, double lz,
          std::size_t max_bytes = kDefaultMaxBytes);
  explicit MacGrid(const Grid3D& grid, std::size_t max_bytes = kDefaultMaxBytes);

  // Bytes needed for p, u, v and w; throws std::invalid_argument on overflow.
  [[nodiscard]] static std::size_t required_bytes(std::size_t nx, std::size_t ny, std::size_t nz);

  [[nodiscard]] std::size_t nx() const noexcept { return nx_; }
  [[nodiscard]] std::size_t ny() const noexcept { return ny_; }
  [[nodiscard]] std::size_t nz() const noexcept { return nz_; }
  [[nodiscard]] double lx() const noexcept { return lx_; }
  [[nodiscard]] double ly() const noexcept { return ly_; }
  [[nodiscard]] double lz() const noexcept { return lz_; }
  [[nodiscard]] double dx() const noexcept { return dx_; }
  [[nodiscard]] double dy() const noexcept { return dy_; }
  [[nodiscard]] double dz() const noexcept { return dz_; }
  [[nodiscard]] std::size_t cell_count() const noexcept { return p_.size(); }

  // True when `grid` has the same cell counts and domain lengths (so the same spacing).
  [[nodiscard]] bool matches(const Grid3D& grid) const noexcept;

  [[nodiscard]] StaggeredArray& p() noexcept { return p_; }
  [[nodiscard]] StaggeredArray& u() noexcept { return u_; }
  [[nodiscard]] StaggeredArray& v() noexcept { return v_; }
  [[nodiscard]] StaggeredArray& w() noexcept { return w_; }
  [[nodiscard]] const StaggeredArray& p() const noexcept { return p_; }
  [[nodiscard]] const StaggeredArray& u() const noexcept { return u_; }
  [[nodiscard]] const StaggeredArray& v() const noexcept { return v_; }
  [[nodiscard]] const StaggeredArray& w() const noexcept { return w_; }

  // Physical locations (meters). Unchecked: indices may address any sample of that field.
  [[nodiscard]] Vec3 p_position(std::size_t i, std::size_t j, std::size_t k) const noexcept;
  [[nodiscard]] Vec3 u_position(std::size_t i, std::size_t j, std::size_t k) const noexcept;
  [[nodiscard]] Vec3 v_position(std::size_t i, std::size_t j, std::size_t k) const noexcept;
  [[nodiscard]] Vec3 w_position(std::size_t i, std::size_t j, std::size_t k) const noexcept;

  // Zero velocity and zero pressure.
  void set_zero() noexcept;

  // Uniform face velocities for testing. ActiveOnly needs `faces` (from classify_faces on the
  // matching mask) and writes 0.0 to blocked faces; this is NOT an obstacle boundary condition.
  // Throws std::invalid_argument if ActiveOnly is requested without matching face types.
  void set_uniform_velocity(const Vec3& velocity, FaceSelection selection, const FaceTypes* faces = nullptr);

 private:
  std::size_t nx_, ny_, nz_;
  double lx_, ly_, lz_;
  double dx_, dy_, dz_;
  StaggeredArray p_, u_, v_, w_;
};

}  // namespace aeroflow
