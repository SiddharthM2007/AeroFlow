#include "aeroflow/ray.hpp"

#include <cmath>
#include <limits>

namespace aeroflow {

namespace {

Vec3 sub(const Vec3& p, const Vec3& q) { return {p.x - q.x, p.y - q.y, p.z - q.z}; }
Vec3 cross(const Vec3& p, const Vec3& q) {
  return {p.y * q.z - p.z * q.y, p.z * q.x - p.x * q.z, p.x * q.y - p.y * q.x};
}
double dot(const Vec3& p, const Vec3& q) { return p.x * q.x + p.y * q.y + p.z * q.z; }
double length(const Vec3& p) { return std::sqrt(dot(p, p)); }

}  // namespace

RayHit intersect_ray_triangle(const Vec3& origin, const Vec3& dir, const Triangle& tri,
                              const RayTolerances& tol) {
  const Vec3 e1 = sub(tri.b, tri.a);
  const Vec3 e2 = sub(tri.c, tri.a);
  const Vec3 pvec = cross(dir, e2);
  const double det = dot(e1, pvec);
  const double scale = length(e1) * length(e2) * length(dir);

  if (std::abs(det) <= tol.parallel * scale) {
    // Ray (nearly) parallel to the plane. If the ray line lies in the plane it may graze the
    // triangle along an edge or through its interior: no well-defined crossing -> Ambiguous.
    const Vec3 n = cross(e1, e2);
    const double n_len = length(n);
    if (n_len == 0.0) return {HitKind::Miss, 0.0};  // degenerate triangle
    const double distance = std::abs(dot(n, sub(origin, tri.a))) / n_len;
    // t is undefined for an in-plane ray; NaN marks it so callers cannot mistake it for a crossing.
    if (distance <= tol.length) return {HitKind::Ambiguous, std::numeric_limits<double>::quiet_NaN()};
    return {HitKind::Miss, 0.0};
  }

  const double inv_det = 1.0 / det;
  const Vec3 tvec = sub(origin, tri.a);
  const double u = dot(tvec, pvec) * inv_det;
  const Vec3 qvec = cross(tvec, e1);
  const double v = dot(dir, qvec) * inv_det;
  const double w = 1.0 - u - v;
  const double t = dot(e2, qvec) * inv_det;

  const double eps = tol.barycentric;
  if (u < -eps || v < -eps || w < -eps) return {HitKind::Miss, t};
  if (u <= eps || v <= eps || w <= eps) return {HitKind::Ambiguous, t};
  return {HitKind::Hit, t};
}

}  // namespace aeroflow
