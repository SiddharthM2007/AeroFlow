#include "aeroflow/boundary.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace aeroflow {

namespace {

VelocityFaceBC from_face(FaceType type, bool on_inlet, bool on_outlet) {
  switch (type) {
    case FaceType::Fluid: return VelocityFaceBC::Interior;
    case FaceType::SolidBoundary: return VelocityFaceBC::SolidWall;
    case FaceType::SolidInterior: return VelocityFaceBC::Inactive;
    case FaceType::DomainBoundary:
      if (on_inlet) return VelocityFaceBC::Inlet;
      if (on_outlet) return VelocityFaceBC::Outlet;
      return VelocityFaceBC::SlipWall;
  }
  return VelocityFaceBC::Inactive;
}

VelocityFaceBC checked(const std::vector<VelocityFaceBC>& faces, std::size_t n0, std::size_t n1, std::size_t n2,
                       std::size_t i, std::size_t j, std::size_t k, const char* name) {
  if (i >= n0 || j >= n1 || k >= n2) {
    throw std::out_of_range(std::string(name) + " face (" + std::to_string(i) + ", " + std::to_string(j) + ", " +
                            std::to_string(k) + ") is out of range");
  }
  return faces[i + n0 * (j + n1 * k)];
}

struct ComponentView {
  const StaggeredArray& values;
  const std::vector<VelocityFaceBC>& bcs;
  int normal_axis;
};

ComponentView view(const MacGrid& mac, const BoundaryMap& map, VelocityComponent c) {
  switch (c) {
    case VelocityComponent::U: return {mac.u(), map.u, 0};
    case VelocityComponent::V: return {mac.v(), map.v, 1};
    case VelocityComponent::W:
    default: return {mac.w(), map.w, 2};
  }
}

}  // namespace

// --- BoundaryMap ----------------------------------------------------------------------------

VelocityFaceBC BoundaryMap::u_at(std::size_t i, std::size_t j, std::size_t k) const {
  return checked(u, nx + 1, ny, nz, i, j, k, "u");
}
VelocityFaceBC BoundaryMap::v_at(std::size_t i, std::size_t j, std::size_t k) const {
  return checked(v, nx, ny + 1, nz, i, j, k, "v");
}
VelocityFaceBC BoundaryMap::w_at(std::size_t i, std::size_t j, std::size_t k) const {
  return checked(w, nx, ny, nz + 1, i, j, k, "w");
}

std::size_t BoundaryMap::count(VelocityFaceBC bc) const noexcept {
  return static_cast<std::size_t>(std::count(u.begin(), u.end(), bc) + std::count(v.begin(), v.end(), bc) +
                                  std::count(w.begin(), w.end(), bc));
}

bool BoundaryMap::matches(const MacGrid& mac) const noexcept {
  return nx == mac.nx() && ny == mac.ny() && nz == mac.nz() && u.size() == mac.u().size() &&
         v.size() == mac.v().size() && w.size() == mac.w().size();
}

BoundaryMap classify_boundaries(const Grid3D& mask) {
  const FaceTypes faces = classify_faces(mask);
  BoundaryMap map;
  map.nx = faces.nx;
  map.ny = faces.ny;
  map.nz = faces.nz;
  map.u.resize(faces.u.size());
  map.v.resize(faces.v.size());
  map.w.resize(faces.w.size());
  const std::size_t nu0 = map.nx + 1;
  for (std::size_t n = 0; n < faces.u.size(); ++n) {
    const std::size_t i = n % nu0;
    map.u[n] = from_face(faces.u[n], i == 0, i == map.nx);
  }
  for (std::size_t n = 0; n < faces.v.size(); ++n) map.v[n] = from_face(faces.v[n], false, false);
  for (std::size_t n = 0; n < faces.w.size(); ++n) map.w[n] = from_face(faces.w[n], false, false);
  return map;
}

void validate_boundaries(const BoundaryMap& map, const BoundaryConditions& bc) {
  if (!std::isfinite(bc.inlet_velocity) || bc.inlet_velocity < 0.0) {
    throw BoundaryError("inlet velocity must be a finite number >= 0 m/s (flow is along +x)");
  }
  if (map.nx == 0 || map.ny == 0 || map.nz == 0 || map.u.size() != (map.nx + 1) * map.ny * map.nz ||
      map.v.size() != map.nx * (map.ny + 1) * map.nz || map.w.size() != map.nx * map.ny * (map.nz + 1)) {
    throw BoundaryError("boundary map extents are inconsistent");
  }
  if (map.count(VelocityFaceBC::Outlet) == 0) {
    throw BoundaryError(
        "no open outlet face at x = Lx: the domain is fully solid or the outlet is blocked, so the "
        "pressure has no reference and inflow cannot leave");
  }
}

void apply_velocity_bcs(MacGrid& mac, const BoundaryMap& map, const BoundaryConditions& bc) {
  apply_prescribed_velocity_bcs(mac, map, bc);

  // Zero-gradient outlet predictor: copy the opposite face of the last cell (already final).
  auto& u = mac.u();
  const std::size_t nx = map.nx;
  for (std::size_t k = 0; k < map.nz; ++k)
    for (std::size_t j = 0; j < map.ny; ++j)
      if (map.u[u.index(nx, j, k)] == VelocityFaceBC::Outlet) u(nx, j, k) = u(nx - 1, j, k);
}

void apply_prescribed_velocity_bcs(MacGrid& mac, const BoundaryMap& map, const BoundaryConditions& bc) {
  validate_boundaries(map, bc);
  if (!map.matches(mac)) throw BoundaryError("boundary map does not match the MAC grid");

  auto prescribe = [](StaggeredArray& field, const std::vector<VelocityFaceBC>& bcs, double inlet) {
    auto data = field.data();
    for (std::size_t n = 0; n < data.size(); ++n) {
      switch (bcs[n]) {
        case VelocityFaceBC::Inlet: data[n] = inlet; break;
        case VelocityFaceBC::SlipWall:
        case VelocityFaceBC::SolidWall:
        case VelocityFaceBC::Inactive: data[n] = 0.0; break;
        case VelocityFaceBC::Interior:
        case VelocityFaceBC::Outlet: break;
      }
    }
  };
  prescribe(mac.u(), map.u, bc.inlet_velocity);
  prescribe(mac.v(), map.v, 0.0);
  prescribe(mac.w(), map.w, 0.0);
}

double tangential_neighbor(const MacGrid& mac, const BoundaryMap& map, VelocityComponent component,
                           std::size_t i, std::size_t j, std::size_t k, int axis, int side) {
  if (!map.matches(mac)) throw BoundaryError("boundary map does not match the MAC grid");
  const ComponentView c = view(mac, map, component);
  if (axis < 0 || axis > 2 || axis == c.normal_axis) {
    throw std::invalid_argument("tangential_neighbor needs an axis different from the component's normal");
  }
  if (side != -1 && side != 1) throw std::invalid_argument("side must be -1 or +1");
  const double value = c.values.at(i, j, k);
  const std::size_t self = c.values.index(i, j, k);
  if (c.bcs[self] == VelocityFaceBC::Inactive) {
    throw std::invalid_argument("velocity sample is inside a solid; it has no tangential neighbors");
  }

  std::size_t idx[3] = {i, j, k};
  const std::size_t extent[3] = {c.values.n0(), c.values.n1(), c.values.n2()};
  const bool leaves_domain = side < 0 ? idx[axis] == 0 : idx[axis] + 1 == extent[axis];
  if (leaves_domain) {
    // Crossing a tunnel wall perpendicular to `axis` (only x walls for v/w; y/z walls are slip).
    if (axis == 0 && side < 0) return -value;  // inlet: zero transverse velocity
    return value;                              // outlet zero-gradient, or free-slip y/z walls
  }
  idx[axis] = side < 0 ? idx[axis] - 1 : idx[axis] + 1;
  const std::size_t neighbor = c.values.index(idx[0], idx[1], idx[2]);
  if (c.bcs[neighbor] == VelocityFaceBC::Inactive) return -value;  // no-slip: 0 on the voxel face
  return c.values.data()[neighbor];
}

BoundaryFluxes boundary_fluxes(const MacGrid& mac, const BoundaryMap& map) {
  if (!map.matches(mac)) throw BoundaryError("boundary map does not match the MAC grid");
  BoundaryFluxes f;
  const double area_u = mac.dy() * mac.dz(), area_v = mac.dx() * mac.dz(), area_w = mac.dx() * mac.dy();
  auto walls = [&](const StaggeredArray& field, const std::vector<VelocityFaceBC>& bcs, double area) {
    const auto data = field.data();
    for (std::size_t n = 0; n < data.size(); ++n) {
      if (bcs[n] == VelocityFaceBC::SlipWall || bcs[n] == VelocityFaceBC::SolidWall ||
          bcs[n] == VelocityFaceBC::Inactive) {
        f.wall_leak += std::abs(data[n]) * area;
      }
    }
  };
  const auto u = mac.u().data();
  for (std::size_t n = 0; n < u.size(); ++n) {
    if (map.u[n] == VelocityFaceBC::Inlet) {
      f.inlet += u[n] * area_u;
      f.inlet_area += area_u;
    } else if (map.u[n] == VelocityFaceBC::Outlet) {
      f.outlet += u[n] * area_u;
      f.outlet_area += area_u;
    }
  }
  walls(mac.u(), map.u, area_u);
  walls(mac.v(), map.v, area_v);
  walls(mac.w(), map.w, area_w);
  return f;
}

}  // namespace aeroflow
