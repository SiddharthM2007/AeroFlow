#pragma once

#include "aeroflow/mesh.hpp"
#include "aeroflow/vec3.hpp"

namespace aeroflow {

enum class HitKind {
  Miss,
  Hit,        // clean crossing strictly inside the triangle
  Ambiguous,  // within tolerance of an edge/vertex, or the ray grazes the triangle's plane
};

struct RayHit {
  HitKind kind = HitKind::Miss;
  // Ray parameter: hit point = origin + t * dir. NaN for an Ambiguous in-plane (grazing) ray.
  double t = 0.0;
};

struct RayTolerances {
  // Barycentric band around triangle edges/vertices treated as ambiguous. Shared edges and
  // vertices land in this band for every adjacent triangle, so no crossing is counted twice.
  double barycentric = 1e-9;
  // |det| <= parallel * |e1| |e2| |dir|  =>  ray (nearly) parallel to the triangle's plane.
  double parallel = 1e-12;
  // Absolute length tolerance (meters) for "ray lies in the triangle's plane".
  double length = 1e-12;
};

// Möller–Trumbore ray/triangle intersection with three-way classification.
//
// Any decision whose outcome could flip under floating-point rounding (point near an edge or
// vertex, ray nearly in the triangle's plane) is reported as Ambiguous instead of Hit/Miss,
// so callers can resolve it deterministically rather than relying on an epsilon in the count.
RayHit intersect_ray_triangle(const Vec3& origin, const Vec3& dir, const Triangle& tri,
                              const RayTolerances& tol);

}  // namespace aeroflow
