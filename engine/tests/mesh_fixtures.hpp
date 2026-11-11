#pragma once
// Deterministic watertight test meshes and the Milestone 2A model transform
// (p_world = T R S U (p_raw - pivot), R = Rx Ry Rz, Euler XYZ in degrees).

#include <array>
#include <cmath>
#include <numbers>
#include <vector>

#include "aeroflow/mesh.hpp"

namespace fixtures {

using aeroflow::Triangle;
using aeroflow::TriangleMesh;
using aeroflow::Vec3;

inline Vec3 sub(const Vec3& p, const Vec3& q) { return {p.x - q.x, p.y - q.y, p.z - q.z}; }
inline Vec3 cross(const Vec3& p, const Vec3& q) {
  return {p.y * q.z - p.z * q.y, p.z * q.x - p.x * q.z, p.x * q.y - p.y * q.x};
}
inline double dot(const Vec3& p, const Vec3& q) { return p.x * q.x + p.y * q.y + p.z * q.z; }

// Builds triangles from an indexed vertex list so shared vertices are bitwise identical.
inline TriangleMesh from_indexed(const std::vector<Vec3>& v, const std::vector<std::array<int, 3>>& f) {
  TriangleMesh m;
  for (const auto& t : f) m.triangles.push_back({v[t[0]], v[t[1]], v[t[2]]});
  return m;
}

// Axis-aligned box with outward winding. Each face is split along the diagonal from its
// (lowest, lowest) to (highest, highest) corner, so +x rays with equal y and z offsets from
// the box minimum pass exactly through the shared diagonal edge of the x faces.
inline TriangleMesh make_box(const Vec3& lo, const Vec3& hi) {
  std::vector<Vec3> v;
  for (int n = 0; n < 8; ++n) v.push_back({n & 1 ? hi.x : lo.x, n & 2 ? hi.y : lo.y, n & 4 ? hi.z : lo.z});
  // Quads listed as (q0, q1, q2, q3) with q0 = low corner and q2 = high corner of the face.
  const int quads[6][4] = {
      {0, 2, 6, 4}, {1, 3, 7, 5},  // x = lo, x = hi   (y, z)
      {0, 4, 5, 1}, {2, 6, 7, 3},  // y = lo, y = hi   (z, x)
      {0, 1, 3, 2}, {4, 5, 7, 6},  // z = lo, z = hi   (x, y)
  };
  const Vec3 center{(lo.x + hi.x) / 2, (lo.y + hi.y) / 2, (lo.z + hi.z) / 2};
  std::vector<std::array<int, 3>> f;
  for (const auto& q : quads) {
    std::array<int, 3> t1{q[0], q[1], q[2]}, t2{q[0], q[2], q[3]};
    const Vec3 n = cross(sub(v[t1[1]], v[t1[0]]), sub(v[t1[2]], v[t1[0]]));
    if (dot(n, sub(v[q[0]], center)) < 0) {  // make winding outward
      std::swap(t1[1], t1[2]);
      std::swap(t2[1], t2[2]);
    }
    f.push_back(t1);
    f.push_back(t2);
  }
  return from_indexed(v, f);
}

// UV sphere with poles on +-y; watertight with exactly shared vertices, outward winding.
inline TriangleMesh make_sphere(const Vec3& c, double r, int stacks, int slices) {
  std::vector<Vec3> v;
  v.push_back({c.x, c.y + r, c.z});
  for (int s = 1; s < stacks; ++s) {
    const double phi = std::numbers::pi * s / stacks;
    for (int k = 0; k < slices; ++k) {
      const double theta = 2 * std::numbers::pi * k / slices;
      v.push_back({c.x + r * std::sin(phi) * std::cos(theta), c.y + r * std::cos(phi),
                   c.z + r * std::sin(phi) * std::sin(theta)});
    }
  }
  v.push_back({c.x, c.y - r, c.z});
  const int south = static_cast<int>(v.size()) - 1;
  auto ring = [&](int s, int k) { return 1 + (s - 1) * slices + (k % slices); };

  std::vector<std::array<int, 3>> f;
  for (int k = 0; k < slices; ++k) f.push_back({0, ring(1, k + 1), ring(1, k)});
  for (int s = 1; s < stacks - 1; ++s) {
    for (int k = 0; k < slices; ++k) {
      f.push_back({ring(s, k), ring(s, k + 1), ring(s + 1, k + 1)});
      f.push_back({ring(s, k), ring(s + 1, k + 1), ring(s + 1, k)});
    }
  }
  for (int k = 0; k < slices; ++k) f.push_back({south, ring(stacks - 1, k), ring(stacks - 1, k + 1)});
  TriangleMesh m = from_indexed(v, f);
  if (aeroflow::signed_volume(m) < 0) {
    for (auto& t : m.triangles) std::swap(t.b, t.c);
  }
  return m;
}

// Octahedron |x-cx| + |y-cy| + |z-cz| <= r, vertices on the axes.
inline TriangleMesh make_octahedron(const Vec3& c, double r) {
  const std::vector<Vec3> v = {{c.x + r, c.y, c.z}, {c.x - r, c.y, c.z}, {c.x, c.y + r, c.z},
                               {c.x, c.y - r, c.z}, {c.x, c.y, c.z + r}, {c.x, c.y, c.z - r}};
  std::vector<std::array<int, 3>> f;
  for (int sx : {0, 1})
    for (int sy : {2, 3})
      for (int sz : {4, 5}) f.push_back({sx, sy, sz});
  TriangleMesh m = from_indexed(v, f);
  for (auto& t : m.triangles) {  // outward winding per face
    const Vec3 n = cross(sub(t.b, t.a), sub(t.c, t.a));
    if (dot(n, sub(t.a, c)) < 0) std::swap(t.b, t.c);
  }
  return m;
}

using Matrix = std::array<double, 12>;  // row-major 3x4 (last row of the 4x4 is 0 0 0 1)

// Same composition as backend/app/transform.py compose_matrix().
inline Matrix compose(double unit, const Vec3& pivot, const Vec3& scale, const Vec3& rot_deg, const Vec3& pos) {
  const double d2r = std::numbers::pi / 180.0;
  const double cx = std::cos(rot_deg.x * d2r), sx = std::sin(rot_deg.x * d2r);
  const double cy = std::cos(rot_deg.y * d2r), sy = std::sin(rot_deg.y * d2r);
  const double cz = std::cos(rot_deg.z * d2r), sz = std::sin(rot_deg.z * d2r);
  const double Rx[3][3] = {{1, 0, 0}, {0, cx, -sx}, {0, sx, cx}};
  const double Ry[3][3] = {{cy, 0, sy}, {0, 1, 0}, {-sy, 0, cy}};
  const double Rz[3][3] = {{cz, -sz, 0}, {sz, cz, 0}, {0, 0, 1}};
  double Rxy[3][3] = {}, R[3][3] = {};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) Rxy[i][j] += Rx[i][k] * Ry[k][j];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) R[i][j] += Rxy[i][k] * Rz[k][j];
  const double s[3] = {scale.x * unit, scale.y * unit, scale.z * unit};
  const double p[3] = {pivot.x, pivot.y, pivot.z};
  const double t[3] = {pos.x, pos.y, pos.z};
  Matrix m{};
  for (int i = 0; i < 3; ++i) {
    double tr = t[i];
    for (int j = 0; j < 3; ++j) {
      m[i * 4 + j] = R[i][j] * s[j];
      tr -= m[i * 4 + j] * p[j];
    }
    m[i * 4 + 3] = tr;
  }
  return m;
}

inline Vec3 apply(const Matrix& m, const Vec3& p) {
  return {m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3], m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
          m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]};
}

inline TriangleMesh transformed(const TriangleMesh& raw, const Matrix& m) {
  TriangleMesh out;
  for (const auto& t : raw.triangles) out.triangles.push_back({apply(m, t.a), apply(m, t.b), apply(m, t.c)});
  return out;
}

}  // namespace fixtures
