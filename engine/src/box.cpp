#include "aeroflow/box.hpp"

namespace aeroflow {

std::size_t mark_solid_box(Grid3D& grid, const Aabb& box) {
  std::size_t marked = 0;
  auto cells = grid.cells();
  for (std::size_t k = 0; k < grid.nz(); ++k) {
    for (std::size_t j = 0; j < grid.ny(); ++j) {
      for (std::size_t i = 0; i < grid.nx(); ++i) {
        if (box.contains_half_open(grid.cell_center(i, j, k))) {
          cells[grid.index(i, j, k)] = CellType::Solid;
          ++marked;
        }
      }
    }
  }
  return marked;
}

}  // namespace aeroflow
