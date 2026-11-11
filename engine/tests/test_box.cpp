#include <gtest/gtest.h>

#include "aeroflow/box.hpp"

using aeroflow::Aabb;
using aeroflow::CellType;
using aeroflow::Grid3D;
using aeroflow::mark_solid_box;

TEST(Aabb, HalfOpenContainment) {
  const Aabb box{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  EXPECT_TRUE(box.contains_half_open({0.0, 0.0, 0.0}));
  EXPECT_TRUE(box.contains_half_open({0.5, 0.999, 0.1}));
  EXPECT_FALSE(box.contains_half_open({1.0, 0.5, 0.5}));
  EXPECT_FALSE(box.contains_half_open({0.5, 1.0, 0.5}));
  EXPECT_FALSE(box.contains_half_open({0.5, 0.5, 1.0}));
  EXPECT_FALSE(box.contains_half_open({-1e-12, 0.5, 0.5}));
}

TEST(SolidBox, MarksExpectedBlockOfCells) {
  // 10^3 grid on a unit cube: centers at 0.05, 0.15, ..., 0.95.
  Grid3D g(10, 10, 10, 1.0, 1.0, 1.0);
  const Aabb box{{0.2, 0.3, 0.4}, {0.5, 0.7, 0.6}};
  // x centers in [0.2, 0.5): 0.25, 0.35, 0.45          -> i = 2..4
  // y centers in [0.3, 0.7): 0.35, 0.45, 0.55, 0.65    -> j = 3..6
  // z centers in [0.4, 0.6): 0.45, 0.55                -> k = 4..5
  const std::size_t marked = mark_solid_box(g, box);
  EXPECT_EQ(marked, 3u * 4u * 2u);
  EXPECT_EQ(g.count(CellType::Solid), 24u);
  EXPECT_EQ(g.count(CellType::Fluid), 1000u - 24u);

  for (std::size_t k = 0; k < 10; ++k) {
    for (std::size_t j = 0; j < 10; ++j) {
      for (std::size_t i = 0; i < 10; ++i) {
        const bool expect_solid = i >= 2 && i <= 4 && j >= 3 && j <= 6 && k >= 4 && k <= 5;
        EXPECT_EQ(g.cell(i, j, k), expect_solid ? CellType::Solid : CellType::Fluid)
            << "cell (" << i << ", " << j << ", " << k << ")";
      }
    }
  }
}

TEST(SolidBox, BoundaryOnCellCenterIsHalfOpen) {
  // dx = 1: centers at 0.5, 1.5, 2.5, 3.5 (exactly representable).
  Grid3D g(4, 4, 4, 4.0, 4.0, 4.0);
  const Aabb box{{1.5, 1.5, 1.5}, {2.5, 2.5, 2.5}};
  EXPECT_EQ(mark_solid_box(g, box), 1u);
  EXPECT_EQ(g.cell(1, 1, 1), CellType::Solid);  // center 1.5 == min: included
  EXPECT_EQ(g.cell(2, 2, 2), CellType::Fluid);  // center 2.5 == max: excluded
}

TEST(SolidBox, AnisotropicSpacing) {
  Grid3D g(8, 4, 2, 4.0, 1.0, 2.0);  // dx = 0.5, dy = 0.25, dz = 1
  const Aabb box{{1.0, 0.0, 0.0}, {2.0, 0.5, 1.0}};
  // x centers 1.25, 1.75 ; y centers 0.125, 0.375 ; z center 0.5
  EXPECT_EQ(mark_solid_box(g, box), 4u);
  EXPECT_EQ(g.cell(2, 0, 0), CellType::Solid);
  EXPECT_EQ(g.cell(3, 1, 0), CellType::Solid);
  EXPECT_EQ(g.cell(4, 0, 0), CellType::Fluid);
  EXPECT_EQ(g.cell(2, 0, 1), CellType::Fluid);
}

TEST(SolidBox, BoxBetweenCentersMarksNothing) {
  Grid3D g(4, 4, 4, 4.0, 4.0, 4.0);
  const Aabb box{{0.6, 0.6, 0.6}, {1.4, 1.4, 1.4}};
  EXPECT_EQ(mark_solid_box(g, box), 0u);
  EXPECT_EQ(g.count(CellType::Solid), 0u);
}

TEST(SolidBox, WholeDomainBoxMarksEverything) {
  Grid3D g(5, 6, 7, 1.0, 2.0, 3.0);
  const Aabb box{{0.0, 0.0, 0.0}, {1.0, 2.0, 3.0}};
  EXPECT_EQ(mark_solid_box(g, box), g.cell_count());
  EXPECT_EQ(g.count(CellType::Fluid), 0u);
}
