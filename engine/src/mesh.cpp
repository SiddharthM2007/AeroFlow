#include "aeroflow/mesh.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace aeroflow {

namespace {

static_assert(std::endian::native == std::endian::little,
              "triangle file I/O assumes a little-endian host");

constexpr std::size_t kHeaderBytes = 16;
constexpr std::size_t kTriangleBytes = 9 * sizeof(double);

Vec3 sub(const Vec3& p, const Vec3& q) { return {p.x - q.x, p.y - q.y, p.z - q.z}; }
Vec3 cross(const Vec3& p, const Vec3& q) {
  return {p.y * q.z - p.z * q.y, p.z * q.x - p.x * q.z, p.x * q.y - p.y * q.x};
}
double dot(const Vec3& p, const Vec3& q) { return p.x * q.x + p.y * q.y + p.z * q.z; }
double norm2(const Vec3& p) { return dot(p, p); }

bool finite(const Vec3& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }

// Exact-equality vertex key (bit patterns, with -0.0 folded into +0.0).
struct VertexKey {
  std::array<std::uint64_t, 3> bits;
  friend bool operator==(const VertexKey&, const VertexKey&) = default;
};

VertexKey key_of(const Vec3& p) {
  auto b = [](double v) { return std::bit_cast<std::uint64_t>(v == 0.0 ? 0.0 : v); };
  return {{b(p.x), b(p.y), b(p.z)}};
}

struct VertexKeyHash {
  std::size_t operator()(const VertexKey& k) const noexcept {
    std::uint64_t h = 1469598103934665603ull;
    for (auto v : k.bits) h = (h ^ v) * 1099511628211ull + (h >> 29);
    return static_cast<std::size_t>(h);
  }
};

struct TriangleKeyHash {
  std::size_t operator()(const std::array<std::uint32_t, 3>& k) const noexcept {
    std::uint64_t h = 1469598103934665603ull;
    for (auto v : k) h = (h ^ v) * 1099511628211ull;
    return static_cast<std::size_t>(h);
  }
};

struct PairHash {
  std::size_t operator()(std::uint64_t v) const noexcept { return std::hash<std::uint64_t>{}(v * 0x9E3779B97F4A7C15ull); }
};

// Zero-area test, relative to the triangle's own scale.
bool degenerate(const Triangle& t) {
  const Vec3 e1 = sub(t.b, t.a);
  const Vec3 e2 = sub(t.c, t.a);
  const Vec3 e3 = sub(t.c, t.b);
  const double longest2 = std::max({norm2(e1), norm2(e2), norm2(e3)});
  if (longest2 == 0.0) return true;
  // |e1 x e2| = 2 * area; compare area against longest edge^2 (i.e. a height ~1e-12 of the length).
  return norm2(cross(e1, e2)) <= 1e-24 * longest2 * longest2;
}

std::string count_message(std::size_t n, const char* what) {
  return std::to_string(n) + " " + what + (n == 1 ? "" : "s");
}

}  // namespace

TriangleMesh read_triangle_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) throw MeshError("cannot open triangle file: " + path);
  const auto size = static_cast<std::uint64_t>(in.tellg());
  in.seekg(0);
  if (size < kHeaderBytes) throw MeshError("triangle file is truncated (no header)");

  char header[kHeaderBytes];
  in.read(header, kHeaderBytes);
  if (std::memcmp(header, kTriangleFileMagic, sizeof(kTriangleFileMagic)) != 0) {
    throw MeshError("triangle file has an invalid header");
  }
  std::uint64_t count = 0;
  std::memcpy(&count, header + 8, sizeof(count));
  if (count > (size - kHeaderBytes) / kTriangleBytes || size != kHeaderBytes + count * kTriangleBytes) {
    throw MeshError("triangle file size does not match its triangle count");
  }

  TriangleMesh mesh;
  mesh.triangles.resize(static_cast<std::size_t>(count));
  static_assert(sizeof(Triangle) == kTriangleBytes);
  in.read(reinterpret_cast<char*>(mesh.triangles.data()), static_cast<std::streamsize>(count * kTriangleBytes));
  if (!in) throw MeshError("failed to read triangle data");
  return mesh;
}

void write_triangle_file(const std::string& path, const TriangleMesh& mesh) {
  std::ofstream out(path, std::ios::binary);
  if (!out) throw MeshError("cannot write triangle file: " + path);
  const std::uint64_t count = mesh.triangles.size();
  out.write(kTriangleFileMagic, sizeof(kTriangleFileMagic));
  out.write(reinterpret_cast<const char*>(&count), sizeof(count));
  out.write(reinterpret_cast<const char*>(mesh.triangles.data()),
            static_cast<std::streamsize>(count * kTriangleBytes));
}

Aabb mesh_bounds(const TriangleMesh& mesh) {
  constexpr double inf = std::numeric_limits<double>::infinity();
  Aabb b{{inf, inf, inf}, {-inf, -inf, -inf}};
  for (const auto& t : mesh.triangles) {
    for (const Vec3* p : {&t.a, &t.b, &t.c}) {
      b.min = {std::min(b.min.x, p->x), std::min(b.min.y, p->y), std::min(b.min.z, p->z)};
      b.max = {std::max(b.max.x, p->x), std::max(b.max.y, p->y), std::max(b.max.z, p->z)};
    }
  }
  return b;
}

double signed_volume(const TriangleMesh& mesh) {
  // Accumulate relative to a reference point to limit cancellation for meshes far from the origin.
  if (mesh.triangles.empty()) return 0.0;
  const Vec3 o = mesh.triangles.front().a;
  double sum = 0.0;
  for (const auto& t : mesh.triangles) {
    sum += dot(sub(t.a, o), cross(sub(t.b, o), sub(t.c, o)));
  }
  return sum / 6.0;
}

MeshReport validate_mesh(TriangleMesh& mesh) {
  MeshReport report;
  report.input_triangles = mesh.triangles.size();
  if (mesh.triangles.empty()) throw MeshError("mesh is empty (no triangles)");

  for (std::size_t n = 0; n < mesh.triangles.size(); ++n) {
    const auto& t = mesh.triangles[n];
    if (!finite(t.a) || !finite(t.b) || !finite(t.c)) {
      throw MeshError("triangle " + std::to_string(n) + " has non-finite coordinates");
    }
  }

  const auto before = mesh.triangles.size();
  std::erase_if(mesh.triangles, degenerate);
  report.degenerate_removed = before - mesh.triangles.size();
  report.triangles = mesh.triangles.size();
  if (mesh.triangles.empty()) throw MeshError("mesh contains only degenerate (zero-area) triangles");
  if (report.degenerate_removed > 0) {
    report.warnings.push_back("removed " + count_message(report.degenerate_removed, "degenerate (zero-area) triangle"));
  }

  // Weld vertices by exact equality.
  std::unordered_map<VertexKey, std::uint32_t, VertexKeyHash> ids;
  ids.reserve(mesh.triangles.size() * 2);
  auto id_of = [&](const Vec3& p) {
    const auto [it, inserted] = ids.try_emplace(key_of(p), static_cast<std::uint32_t>(ids.size()));
    return it->second;
  };

  // Edge key (lo, hi) -> {uses, signed direction sum}.
  struct EdgeUse {
    std::uint32_t count = 0;
    std::int32_t direction = 0;
  };
  std::unordered_map<std::uint64_t, EdgeUse, PairHash> edges;
  edges.reserve(mesh.triangles.size() * 2);
  std::unordered_set<std::array<std::uint32_t, 3>, TriangleKeyHash> seen_triangles;
  seen_triangles.reserve(mesh.triangles.size());

  for (const auto& t : mesh.triangles) {
    const std::uint32_t v[3] = {id_of(t.a), id_of(t.b), id_of(t.c)};

    std::array<std::uint32_t, 3> sorted = {v[0], v[1], v[2]};
    std::sort(sorted.begin(), sorted.end());
    if (!seen_triangles.insert(sorted).second) ++report.duplicate_triangles;

    for (int e = 0; e < 3; ++e) {
      const std::uint32_t p = v[e];
      const std::uint32_t q = v[(e + 1) % 3];
      const std::uint64_t key = p < q ? (std::uint64_t{p} << 32 | q) : (std::uint64_t{q} << 32 | p);
      auto& use = edges[key];
      ++use.count;
      use.direction += p < q ? 1 : -1;
    }
  }
  report.unique_vertices = ids.size();

  for (const auto& [_, use] : edges) {
    if (use.count == 1) {
      ++report.open_edges;
    } else if (use.count > 2) {
      ++report.nonmanifold_edges;
    } else if (use.direction != 0) {
      ++report.inconsistent_edges;
    }
  }
  report.consistently_oriented = report.inconsistent_edges == 0;
  report.bounds = mesh_bounds(mesh);

  if (report.duplicate_triangles > 0) {
    throw MeshError("mesh has " + count_message(report.duplicate_triangles, "duplicate triangle") +
                    "; duplicated faces break inside/outside parity");
  }
  if (report.nonmanifold_edges > 0) {
    throw MeshError("mesh is non-manifold: " + count_message(report.nonmanifold_edges, "edge") +
                    " shared by more than two triangles");
  }
  if (report.open_edges > 0) {
    throw MeshError("mesh is not watertight: " + count_message(report.open_edges, "open boundary edge") +
                    " (vertices are matched exactly; close gaps or repair the mesh)");
  }
  if (!report.consistently_oriented) {
    report.warnings.push_back("inconsistent triangle winding on " +
                              count_message(report.inconsistent_edges, "edge") +
                              "; classification is unaffected but the mesh volume is not reported");
  }
  return report;
}

}  // namespace aeroflow
