#include <gtest/gtest.h>

#include <set>
#include <stdexcept>

#include "aeroflow/grid3d.hpp"

using aeroflow::CellType;
using aeroflow::Grid3D;

TEST(Grid3DConstruction, StoresDimensionsAndSpacing) {
  const Grid3D g(4, 5, 6, 2.0, 1.0, 3.0);
  EXPECT_EQ(g.nx(), 4u);
  EXPECT_EQ(g.ny(), 5u);
  EXPECT_EQ(g.nz(), 6u);
  EXPECT_DOUBLE_EQ(g.lx(), 2.0);
  EXPECT_DOUBLE_EQ(g.ly(), 1.0);
  EXPECT_DOUBLE_EQ(g.lz(), 3.0);
  EXPECT_DOUBLE_EQ(g.dx(), 0.5);
  EXPECT_DOUBLE_EQ(g.dy(), 0.2);
  EXPECT_DOUBLE_EQ(g.dz(), 0.5);
  EXPECT_DOUBLE_EQ(g.cell_volume(), 0.5 * 0.2 * 0.5);
  EXPECT_EQ(g.cell_count(), 4u * 5u * 6u);
}

TEST(Grid3DConstruction, InitialisesAllCellsFluid) {
  const Grid3D g(3, 3, 3, 1.0, 1.0, 1.0);
  EXPECT_EQ(g.count(CellType::Fluid), 27u);
  EXPECT_EQ(g.count(CellType::Solid), 0u);
}

TEST(Grid3DConstruction, RejectsInvalidArguments) {
  EXPECT_THROW(Grid3D(0, 1, 1, 1.0, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(Grid3D(1, 0, 1, 1.0, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(Grid3D(1, 1, 0, 1.0, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(Grid3D(1, 1, 1, 0.0, 1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(Grid3D(1, 1, 1, 1.0, -1.0, 1.0), std::invalid_argument);
  EXPECT_THROW(Grid3D(1, 1, 1, 1.0, 1.0, std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_THROW(Grid3D(1, 1, 1, std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0),
               std::invalid_argument);
  EXPECT_THROW(Grid3D(1024, 1024, 1024, 1.0, 1.0, 1.0), std::invalid_argument);
  constexpr auto huge = std::numeric_limits<std::size_t>::max() / 2;
  EXPECT_THROW(Grid3D(huge, huge, 2, 1.0, 1.0, 1.0), std::invalid_argument);
}

TEST(Grid3DIndexing, MatchesXFastestFormula) {
  const Grid3D g(3, 4, 5, 1.0, 1.0, 1.0);
  EXPECT_EQ(g.index(0, 0, 0), 0u);
  EXPECT_EQ(g.index(1, 0, 0), 1u);
  EXPECT_EQ(g.index(0, 1, 0), 3u);
  EXPECT_EQ(g.index(0, 0, 1), 12u);
  EXPECT_EQ(g.index(2, 3, 4), g.cell_count() - 1);
  EXPECT_EQ(g.index(1, 2, 3), 1u + 3u * (2u + 4u * 3u));
}

TEST(Grid3DIndexing, IsBijectiveAndRoundTrips) {
  const Grid3D g(7, 3, 5, 1.0, 1.0, 1.0);
  std::set<std::size_t> seen;
  std::size_t expected = 0;
  for (std::size_t k = 0; k < g.nz(); ++k) {
    for (std::size_t j = 0; j < g.ny(); ++j) {
      for (std::size_t i = 0; i < g.nx(); ++i) {
        const std::size_t idx = g.index(i, j, k);
        // Iterating k, j, i (outer to inner) must visit memory sequentially.
        EXPECT_EQ(idx, expected++);
        seen.insert(idx);
        const auto [ri, rj, rk] = g.unravel(idx);
        EXPECT_EQ(ri, i);
        EXPECT_EQ(rj, j);
        EXPECT_EQ(rk, k);
      }
    }
  }
  EXPECT_EQ(seen.size(), g.cell_count());
}

TEST(Grid3DIndexing, CheckedAccessorsRejectOutOfRange) {
  Grid3D g(2, 2, 2, 1.0, 1.0, 1.0);
  EXPECT_TRUE(g.in_bounds(1, 1, 1));
  EXPECT_FALSE(g.in_bounds(2, 0, 0));
  EXPECT_THROW((void)g.cell(2, 0, 0), std::out_of_range);
  EXPECT_THROW((void)g.cell(0, 2, 0), std::out_of_range);
  EXPECT_THROW(g.set_cell(0, 0, 2, CellType::Solid), std::out_of_range);
}

TEST(Grid3DStorage, SetCellWritesContiguousSlot) {
  Grid3D g(3, 4, 5, 1.0, 1.0, 1.0);
  g.set_cell(1, 2, 3, CellType::Solid);
  EXPECT_EQ(g.cell(1, 2, 3), CellType::Solid);
  EXPECT_EQ(g.cells()[g.index(1, 2, 3)], CellType::Solid);
  EXPECT_EQ(g.count(CellType::Solid), 1u);
  // Storage is a single contiguous block.
  EXPECT_EQ(&g.cells()[g.cell_count() - 1] - &g.cells()[0],
            static_cast<std::ptrdiff_t>(g.cell_count() - 1));
  g.fill(CellType::Fluid);
  EXPECT_EQ(g.count(CellType::Solid), 0u);
}

TEST(Grid3DCoordinates, CellCentersAreOffsetByHalfSpacing) {
  const Grid3D g(4, 2, 5, 2.0, 1.0, 10.0);  // dx = 0.5, dy = 0.5, dz = 2
  EXPECT_DOUBLE_EQ(g.cell_center_x(0), 0.25);
  EXPECT_DOUBLE_EQ(g.cell_center_x(3), 1.75);
  EXPECT_DOUBLE_EQ(g.cell_center_y(0), 0.25);
  EXPECT_DOUBLE_EQ(g.cell_center_y(1), 0.75);
  EXPECT_DOUBLE_EQ(g.cell_center_z(0), 1.0);
  EXPECT_DOUBLE_EQ(g.cell_center_z(4), 9.0);

  const auto c = g.cell_center(2, 1, 3);
  EXPECT_DOUBLE_EQ(c.x, 1.25);
  EXPECT_DOUBLE_EQ(c.y, 0.75);
  EXPECT_DOUBLE_EQ(c.z, 7.0);
}

TEST(Grid3DCoordinates, CentersAreSymmetricAndInsideDomain) {
  const Grid3D g(7, 9, 11, 3.0, 1.7, 0.9);
  for (std::size_t i = 0; i < g.nx(); ++i) {
    const double x = g.cell_center_x(i);
    EXPECT_GT(x, 0.0);
    EXPECT_LT(x, g.lx());
    // Mirror symmetry about the domain mid-plane.
    EXPECT_NEAR(x + g.cell_center_x(g.nx() - 1 - i), g.lx(), 1e-12);
  }
  for (std::size_t k = 0; k + 1 < g.nz(); ++k) {
    EXPECT_NEAR(g.cell_center_z(k + 1) - g.cell_center_z(k), g.dz(), 1e-12);
  }
}

TEST(Grid3DCoordinates, SingleCellCenterIsDomainMidpoint) {
  const Grid3D g(1, 1, 1, 2.0, 4.0, 6.0);
  const auto c = g.cell_center(0, 0, 0);
  EXPECT_DOUBLE_EQ(c.x, 1.0);
  EXPECT_DOUBLE_EQ(c.y, 2.0);
  EXPECT_DOUBLE_EQ(c.z, 3.0);
}
