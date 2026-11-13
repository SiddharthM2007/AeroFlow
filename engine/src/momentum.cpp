#include "aeroflow/momentum.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aeroflow {

namespace {

struct Component {
  const StaggeredArray& f;
  const std::vector<VelocityFaceBC>& bc;
  int axis;
};

Component component_of(const MacGrid& mac, const BoundaryMap& map, int axis) {
  if (axis == 0) return {mac.u(), map.u, 0};
  if (axis == 1) return {mac.v(), map.v, 1};
  return {mac.w(), map.w, 2};
}

int axis_of(VelocityComponent c) { return c == VelocityComponent::U ? 0 : c == VelocityComponent::V ? 1 : 2; }

// Neighbor with the boundary rules documented in momentum.hpp (no bounds checks: hot path).
inline double neighbor(const Component& c, const std::size_t (&idx)[3], int b, int side, double value) {
  const std::size_t ext[3] = {c.f.n0(), c.f.n1(), c.f.n2()};
  const bool leaves = side < 0 ? idx[b] == 0 : idx[b] + 1 == ext[b];
  if (leaves) {
    if (b != c.axis && b == 0 && side < 0) return -value;  // inlet: zero transverse velocity
    return value;                                          // slip walls, outlet: zero gradient
  }
  std::size_t n[3] = {idx[0], idx[1], idx[2]};
  n[b] = side < 0 ? n[b] - 1 : n[b] + 1;
  const std::size_t lin = c.f.index(n[0], n[1], n[2]);
  if (b != c.axis && c.bc[lin] == VelocityFaceBC::Inactive) return -value;  // no-slip on the voxel face
  return c.f.data()[lin];
}

// Average of the 4 samples of component `d` around a sample of component `c` (both Interior-safe).
inline double transverse_average(const Component& c, const Component& d, const std::size_t (&idx)[3]) {
  const int a = c.axis, e = d.axis;
  double sum = 0.0;
  for (int da = -1; da <= 0; ++da)
    for (int de = 0; de <= 1; ++de) {
      std::size_t n[3] = {idx[0], idx[1], idx[2]};
      n[a] = static_cast<std::size_t>(static_cast<std::ptrdiff_t>(n[a]) + da);
      n[e] += static_cast<std::size_t>(de);
      sum += d.f(n[0], n[1], n[2]);
    }
  return 0.25 * sum;
}

inline double advection_at(const MacGrid& mac, const BoundaryMap& map, int axis, const std::size_t (&idx)[3],
                           const double (&h)[3]) {
  const Component c = component_of(mac, map, axis);
  const double value = c.f(idx[0], idx[1], idx[2]);
  double sum = 0.0;
  for (int b = 0; b < 3; ++b) {
    const double q = b == axis ? value : transverse_average(c, component_of(mac, map, b), idx);
    if (q > 0.0) sum += q * (value - neighbor(c, idx, b, -1, value)) / h[b];
    else if (q < 0.0) sum += q * (neighbor(c, idx, b, +1, value) - value) / h[b];
  }
  return -sum;
}

inline double laplacian_at(const MacGrid& mac, const BoundaryMap& map, int axis, const std::size_t (&idx)[3],
                           const double (&h2)[3]) {
  const Component c = component_of(mac, map, axis);
  const double value = c.f(idx[0], idx[1], idx[2]);
  double sum = 0.0;
  for (int b = 0; b < 3; ++b) {
    sum += (neighbor(c, idx, b, +1, value) - 2.0 * value + neighbor(c, idx, b, -1, value)) / h2[b];
  }
  return sum;
}

void require_interior(const MacGrid& mac, const BoundaryMap& map, int axis, std::size_t i, std::size_t j,
                      std::size_t k) {
  if (!map.matches(mac)) throw BoundaryError("boundary map does not match the MAC grid");
  const Component c = component_of(mac, map, axis);
  if (!c.f.in_bounds(i, j, k)) throw std::out_of_range("velocity sample is out of range");
  if (c.bc[c.f.index(i, j, k)] != VelocityFaceBC::Interior) {
    throw std::invalid_argument("momentum terms are defined on Interior faces only");
  }
}

}  // namespace

double velocity_neighbor(const MacGrid& mac, const BoundaryMap& map, VelocityComponent component, std::size_t i,
                         std::size_t j, std::size_t k, int axis, int side) {
  if (!map.matches(mac)) throw BoundaryError("boundary map does not match the MAC grid");
  if (axis < 0 || axis > 2 || (side != -1 && side != 1)) throw std::invalid_argument("bad axis or side");
  const Component c = component_of(mac, map, axis_of(component));
  const std::size_t idx[3] = {i, j, k};
  return neighbor(c, idx, axis, side, c.f.at(i, j, k));
}

double advection_term(const MacGrid& mac, const BoundaryMap& map, VelocityComponent component, std::size_t i,
                      std::size_t j, std::size_t k) {
  const int axis = axis_of(component);
  require_interior(mac, map, axis, i, j, k);
  const double h[3] = {mac.dx(), mac.dy(), mac.dz()};
  const std::size_t idx[3] = {i, j, k};
  return advection_at(mac, map, axis, idx, h);
}

double laplacian_term(const MacGrid& mac, const BoundaryMap& map, VelocityComponent component, std::size_t i,
                      std::size_t j, std::size_t k) {
  const int axis = axis_of(component);
  require_interior(mac, map, axis, i, j, k);
  const double h2[3] = {mac.dx() * mac.dx(), mac.dy() * mac.dy(), mac.dz() * mac.dz()};
  const std::size_t idx[3] = {i, j, k};
  return laplacian_at(mac, map, axis, idx, h2);
}

void explicit_momentum_update(const MacGrid& mac, const BoundaryMap& map, const MomentumTerms& terms, double dt,
                              StaggeredArray& out_u, StaggeredArray& out_v, StaggeredArray& out_w) {
  if (!map.matches(mac)) throw BoundaryError("boundary map does not match the MAC grid");
  if (out_u.size() != mac.u().size() || out_v.size() != mac.v().size() || out_w.size() != mac.w().size()) {
    throw std::invalid_argument("momentum output arrays do not match the MAC grid");
  }
  const double h[3] = {mac.dx(), mac.dy(), mac.dz()};
  const double h2[3] = {h[0] * h[0], h[1] * h[1], h[2] * h[2]};
  StaggeredArray* outs[3] = {&out_u, &out_v, &out_w};
  for (int axis = 0; axis < 3; ++axis) {
    const Component c = component_of(mac, map, axis);
    StaggeredArray& out = *outs[axis];
    const std::size_t n0 = c.f.n0(), n1 = c.f.n1(), n2 = c.f.n2();
    for (std::size_t k = 0; k < n2; ++k)
      for (std::size_t j = 0; j < n1; ++j)
        for (std::size_t i = 0; i < n0; ++i) {
          const std::size_t lin = c.f.index(i, j, k);
          const double value = c.f.data()[lin];
          if (c.bc[lin] != VelocityFaceBC::Interior) {
            out.data()[lin] = value;
            continue;
          }
          const std::size_t idx[3] = {i, j, k};
          double tendency = 0.0;
          if (terms.advection) tendency += advection_at(mac, map, axis, idx, h);
          if (terms.diffusion && terms.viscosity > 0.0) tendency += terms.viscosity * laplacian_at(mac, map, axis, idx, h2);
          out.data()[lin] = value + dt * tendency;
        }
  }
}

VelocityBounds velocity_bounds(const MacGrid& mac) {
  VelocityBounds b;
  auto scan = [&](const StaggeredArray& f, double& m) {
    for (double v : f.data()) {
      if (!std::isfinite(v)) b.finite = false;
      else m = std::max(m, std::abs(v));
    }
  };
  scan(mac.u(), b.max_u);
  scan(mac.v(), b.max_v);
  scan(mac.w(), b.max_w);
  return b;
}

double stability_limit(const MacGrid& mac, double viscosity) {
  const VelocityBounds b = velocity_bounds(mac);
  const double rate = b.max_u / mac.dx() + b.max_v / mac.dy() + b.max_w / mac.dz() +
                      2.0 * viscosity * (1.0 / (mac.dx() * mac.dx()) + 1.0 / (mac.dy() * mac.dy()) +
                                         1.0 / (mac.dz() * mac.dz()));
  return rate > 0.0 ? 1.0 / rate : std::numeric_limits<double>::infinity();
}

}  // namespace aeroflow
