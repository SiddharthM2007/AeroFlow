#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "aeroflow/box.hpp"
#include "aeroflow/vec3.hpp"

namespace aeroflow {

// Raised for unreadable or invalid triangle meshes. Messages are user-facing.
class MeshError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct Triangle {
  Vec3 a;
  Vec3 b;
  Vec3 c;
};

// World-space triangle soup in meters (tunnel frame: origin at the tunnel corner, flow +x, y up).
struct TriangleMesh {
  std::vector<Triangle> triangles;
};

// Binary world-space triangle file (the backend -> engine geometry boundary), little-endian:
//   bytes 0..7   magic "AFTRI1\0\0"
//   bytes 8..15  uint64 triangle count n
//   then n x 9 float64: v0.x v0.y v0.z v1.x v1.y v1.z v2.x v2.y v2.z  (meters)
// Vertices are already transformed (p_world = T R S U (p_raw - pivot)); the engine applies no
// further transformation.
inline constexpr char kTriangleFileMagic[8] = {'A', 'F', 'T', 'R', 'I', '1', '\0', '\0'};

TriangleMesh read_triangle_file(const std::string& path);
void write_triangle_file(const std::string& path, const TriangleMesh& mesh);

struct MeshReport {
  std::size_t input_triangles = 0;
  std::size_t triangles = 0;  // after removing degenerate triangles
  std::size_t degenerate_removed = 0;
  std::size_t unique_vertices = 0;
  std::size_t open_edges = 0;
  std::size_t nonmanifold_edges = 0;
  std::size_t duplicate_triangles = 0;
  std::size_t inconsistent_edges = 0;  // shared edges traversed in the same direction twice
  bool consistently_oriented = true;
  Aabb bounds{};
  std::vector<std::string> warnings;
};

// Validates `mesh` for ray-parity voxelization and removes degenerate (zero-area) triangles.
//
// Vertices are welded by exact coordinate equality (+0.0 == -0.0); no tolerance welding, so
// CAD gaps appear as open edges instead of being silently closed.
//
// Throws MeshError when the mesh is empty, has non-finite coordinates, duplicate triangles,
// open (boundary) edges or non-manifold edges. Inconsistent winding and removed degenerate
// triangles are reported as warnings (winding does not affect parity classification).
MeshReport validate_mesh(TriangleMesh& mesh);

// Enclosed volume via signed tetrahedra: V = 1/6 * sum v0 . (v1 x v2).
// Positive for outward-facing (counter-clockwise seen from outside) winding.
double signed_volume(const TriangleMesh& mesh);

Aabb mesh_bounds(const TriangleMesh& mesh);

}  // namespace aeroflow
