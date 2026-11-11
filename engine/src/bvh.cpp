#include "aeroflow/bvh.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace aeroflow {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

void grow(Aabb& box, const Vec3& p) {
  box.min = {std::min(box.min.x, p.x), std::min(box.min.y, p.y), std::min(box.min.z, p.z)};
  box.max = {std::max(box.max.x, p.x), std::max(box.max.y, p.y), std::max(box.max.z, p.z)};
}

double component(const Vec3& p, int axis) { return axis == 0 ? p.x : axis == 1 ? p.y : p.z; }

// The traversal stack holds at most (depth + 1) entries; median splits keep depth ~ log2(n).
constexpr std::size_t kMaxDepth = 100;

}  // namespace

bool ray_hits_box(const Vec3& origin, const Vec3& dir, double t_min, const Aabb& box) noexcept {
  double t0 = t_min;
  double t1 = kInf;
  for (int a = 0; a < 3; ++a) {
    const double o = component(origin, a);
    const double d = component(dir, a);
    const double lo = component(box.min, a);
    const double hi = component(box.max, a);
    if (d == 0.0) {
      if (o < lo || o > hi) return false;
      continue;
    }
    double ta = (lo - o) / d;
    double tb = (hi - o) / d;
    if (ta > tb) std::swap(ta, tb);
    t0 = std::max(t0, ta);
    t1 = std::min(t1, tb);
    if (t0 > t1) return false;
  }
  return true;
}

Bvh::Bvh(const TriangleMesh& mesh, std::size_t leaf_size, double padding)
    : mesh_(mesh), leaf_size_(std::max<std::size_t>(1, leaf_size)), padding_(padding) {
  const std::size_t n = mesh.triangles.size();
  if (n == 0) return;
  if (n > std::numeric_limits<std::uint32_t>::max() / 2) throw std::length_error("too many triangles for BVH");

  order_.resize(n);
  centroids_.resize(n);
  for (std::uint32_t i = 0; i < n; ++i) {
    order_[i] = i;
    const auto& t = mesh.triangles[i];
    centroids_[i] = {(t.a.x + t.b.x + t.c.x) / 3.0, (t.a.y + t.b.y + t.c.y) / 3.0, (t.a.z + t.b.z + t.c.z) / 3.0};
  }
  nodes_.reserve(2 * n / leaf_size_ + 1);
  nodes_.emplace_back();
  build_into(0, 0, static_cast<std::uint32_t>(n), 0);
  centroids_.clear();
  centroids_.shrink_to_fit();
}

void Bvh::build_into(std::uint32_t index, std::uint32_t begin, std::uint32_t end, std::size_t depth) {
  depth_ = std::max(depth_, depth);

  Aabb box{{kInf, kInf, kInf}, {-kInf, -kInf, -kInf}};
  Aabb centroid_box = box;
  for (std::uint32_t n = begin; n < end; ++n) {
    const auto& t = mesh_.triangles[order_[n]];
    grow(box, t.a);
    grow(box, t.b);
    grow(box, t.c);
    grow(centroid_box, centroids_[order_[n]]);
  }
  box.min = {box.min.x - padding_, box.min.y - padding_, box.min.z - padding_};
  box.max = {box.max.x + padding_, box.max.y + padding_, box.max.z + padding_};
  nodes_[index].box = box;

  const std::uint32_t count = end - begin;
  const Vec3 extent = {centroid_box.max.x - centroid_box.min.x, centroid_box.max.y - centroid_box.min.y,
                       centroid_box.max.z - centroid_box.min.z};
  const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : (extent.y >= extent.z ? 1 : 2);
  if (count <= leaf_size_ || component(extent, axis) <= 0.0 || depth >= kMaxDepth) {
    nodes_[index].first = begin;
    nodes_[index].count = count;
    return;
  }

  const std::uint32_t mid = begin + count / 2;
  std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                   [&](std::uint32_t l, std::uint32_t r) {
                     const double cl = component(centroids_[l], axis);
                     const double cr = component(centroids_[r], axis);
                     return cl < cr || (cl == cr && l < r);  // deterministic tie-break
                   });

  // Children are stored adjacently: left = first, right = first + 1.
  const auto left = static_cast<std::uint32_t>(nodes_.size());
  nodes_.emplace_back();
  nodes_.emplace_back();
  nodes_[index].first = left;
  nodes_[index].count = 0;
  build_into(left, begin, mid, depth + 1);
  build_into(left + 1, mid, end, depth + 1);
}

}  // namespace aeroflow
