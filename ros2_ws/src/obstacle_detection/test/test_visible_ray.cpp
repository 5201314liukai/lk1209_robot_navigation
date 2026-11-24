#include <algorithm>
#include <vector>
#include <gtest/gtest.h>
#include "obstacle_detection/visible_ray.hpp"

using obstacle_detection::GridCell;
using obstacle_detection::GridPoint;
using obstacle_detection::visible_ray;

TEST(VisibleRay, DownwardRingClearsOldBarSurfaceOnlyAfterRemoval)
{
  // World x-cell 5 cm, z-cell 2.5 cm. An obstacle used to terminate
  // the -4 degree ray at x=0.76 m. The farther floor ray crosses that voxel.
  const GridPoint origin{30.,30.,9.2};
  // Azimuth column 166/360, elevation -4 degrees. At x=.76 m the beam
  // enters a sliver of cell (45,26,7) near its bottom face.
  const GridPoint delta{2.5*.969848/.05, 2.5*(-.233513)/.05, 2.5*(-.069756)/.025};
  const GridCell size{60,60,16}, old_bar{45,26,7};
  const auto hits = [&](double limit) {
      std::vector<GridCell> cells;
      visible_ray(origin, delta, size, .048, limit, [&](GridCell c) {cells.push_back(c);});
      return std::find(cells.begin(), cells.end(), old_bar) != cells.end();
    };
  EXPECT_FALSE(hits(.66/2.5));  // low bar is still in front of the sensor
  EXPECT_TRUE(hits(.96));        // the same beam now observes background
}

TEST(VisibleRay, EdgeCrossingsExcludeCornerOnlyCellsAndRespectBounds)
{
  std::vector<GridCell> seen;
  visible_ray({2.,2.,.5}, {-4.,-4.,0.}, {3,3,2}, 0.,1.,
    [&](GridCell c) {seen.push_back(c);});
  EXPECT_EQ(seen, (std::vector<GridCell>{{1,1,0},{0,0,0}}));
}

TEST(VisibleRay, RespectsBlindZoneAndEndpoint)
{
  std::vector<GridCell> seen;
  visible_ray({0.,.5,.5}, {8.,0.,0.}, {8,2,2}, .25,.5,
    [&](GridCell c) {seen.push_back(c);});
  EXPECT_EQ(seen, (std::vector<GridCell>{{2,0,0},{3,0,0}}));
}
