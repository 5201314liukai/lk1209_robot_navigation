#ifndef LK1209_ASTAR_PLANNER__GRID_HPP_
#define LK1209_ASTAR_PLANNER__GRID_HPP_

#include <cstddef>
#include <vector>

namespace lk1209_astar_planner
{
// Indices are row-major (y * width + x). Unreachable endpoints return an empty path.
std::vector<std::size_t> findPath(
  const unsigned char * costs, std::size_t width, std::size_t height,
  std::size_t start, std::size_t goal, double cost_weight);
}  // namespace lk1209_astar_planner

#endif
