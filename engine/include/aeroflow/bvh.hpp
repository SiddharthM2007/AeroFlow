#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "aeroflow/box.hpp"
#include "aeroflow/mesh.hpp"

namespace aeroflow {

// Bounding volume hierarchy over a triangle mesh (median split on the longest centroid axis).
class Bvh {
 public:
  struct Node {
    Aabb box;
    std::uint32_t first = 0;  // leaf: first index into triangle_order(); inner: left child index
    std::uint32_t count = 0;  // leaf: triangle count (> 0); inner: 0 (right child = first + 1)
  };

  // `padding` (meters) inflates every node box so rays touching a box boundary are not culled.
  Bvh(const TriangleMesh& mesh, std::size_t leaf_size, double padding);

  // Calls visit(triangle_index) for every triangle whose leaf box the ray [t_min, inf) enters.
  // Returns the number of ray/box tests performed.
  template <typename Visit>
  std::size_t traverse(const Vec3& origin, const Vec3& dir, double t_min, Visit&& visit) const;

  [[nodiscard]] const std::vector<Node>& nodes() const noexcept { return nodes_; }
  [[nodiscard]] const std::vector<std::uint32_t>& triangle_order() const noexcept { return order_; }
  [[nodiscard]] std::size_t depth() const noexcept { return depth_; }

 private:
  void build_into(std::uint32_t index, std::uint32_t begin, std::uint32_t end, std::size_t depth);

  const TriangleMesh& mesh_;
  std::size_t leaf_size_;
  double padding_;
  std::vector<Node> nodes_;
  std::vector<std::uint32_t> order_;
  std::vector<Vec3> centroids_;
  std::size_t depth_ = 0;
};

// Slab test for the ray [t_min, inf). Direction components of exactly zero are handled without
// division (the origin must then lie within that slab).
bool ray_hits_box(const Vec3& origin, const Vec3& dir, double t_min, const Aabb& box) noexcept;

template <typename Visit>
std::size_t Bvh::traverse(const Vec3& origin, const Vec3& dir, double t_min, Visit&& visit) const {
  std::size_t box_tests = 0;
  if (nodes_.empty()) return 0;
  std::uint32_t stack[128];
  std::size_t top = 0;
  stack[top++] = 0;
  while (top > 0) {
    const Node& node = nodes_[stack[--top]];
    ++box_tests;
    if (!ray_hits_box(origin, dir, t_min, node.box)) continue;
    if (node.count > 0) {
      for (std::uint32_t n = node.first; n < node.first + node.count; ++n) visit(order_[n]);
    } else {
      stack[top++] = node.first;
      stack[top++] = node.first + 1;
    }
  }
  return box_tests;
}

}  // namespace aeroflow
