#pragma once

#include <cstddef>

#include "aeroflow/grid3d.hpp"
#include "aeroflow/vec3.hpp"

namespace aeroflow {

// Axis-aligned box used as a procedural test solid (no voxelization yet).
struct Aabb {
  Vec3 min;
  Vec3 max;

  // Half-open containment: min <= p < max on every axis.
  [[nodiscard]] bool contains_half_open(const Vec3& p) const noexcept {
    return p.x >= min.x && p.x < max.x &&  //
           p.y >= min.y && p.y < max.y &&  //
           p.z >= min.z && p.z < max.z;
  }
};

// Marks every cell whose center lies in the half-open box [min, max) as Solid.
// Other cells are left untouched. Returns the number of cells marked.
std::size_t mark_solid_box(Grid3D& grid, const Aabb& box);

}  // namespace aeroflow
