#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <vector>

#include "lk1209_astar_planner/grid.hpp"

using lk1209_astar_planner::findPath;
using Route = std::vector<std::size_t>;

TEST(FourNeighborAStar, FindsShortestCardinalDetour)
{
  const std::vector<unsigned char> costs = {0, 254, 0, 0, 0, 0};
  EXPECT_EQ(findPath(costs.data(), 3, 2, 0, 2, 4.0), (Route{0, 3, 4, 5, 2}));
}

TEST(FourNeighborAStar, PrefersLongerRouteWhenInflationIsCostly)
{
  const std::vector<unsigned char> costs = {0, 252, 0, 0, 0, 0};
  EXPECT_EQ(findPath(costs.data(), 3, 2, 0, 2, 0.0), (Route{0, 1, 2}));
  EXPECT_EQ(findPath(costs.data(), 3, 2, 0, 2, 4.0), (Route{0, 3, 4, 5, 2}));
}

TEST(FourNeighborAStar, NeverWrapsAtRowEndsOrCrossesForbiddenCells)
{
  for (unsigned char blocked : {253, 254, 255}) {
    const std::vector<unsigned char> costs = {blocked, blocked, 0, 0, blocked, blocked};
    EXPECT_TRUE(findPath(costs.data(), 3, 2, 2, 3, 0.0).empty());
    EXPECT_TRUE(findPath(costs.data(), 3, 2, 0, 0, 0.0).empty());
    EXPECT_TRUE(findPath(costs.data(), 3, 2, 2, 0, 0.0).empty());
  }
}

TEST(FourNeighborAStar, HandlesCoincidentAndOutOfBoundsEndpoints)
{
  const std::vector<unsigned char> costs = {0};
  EXPECT_EQ(findPath(costs.data(), 1, 1, 0, 0, 4.0), (Route{0}));
  EXPECT_TRUE(findPath(costs.data(), 1, 1, 0, 1, 4.0).empty());
  EXPECT_TRUE(findPath(costs.data(), 1, 1, 1, 0, 4.0).empty());
}
