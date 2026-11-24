#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace obstacle_detection
{
using GridPoint = std::array<double, 3>;
using GridCell = std::array<int, 3>;

// Positions are continuous voxel coordinates, so each axis can have its own
// resolution. Visit only voxels with a positive-length visible intersection.
// The caller verifies that the original sensor position is inside the grid.
template<class Visit>
double visible_ray(const GridPoint &start, const GridPoint &delta, const GridCell &size,
                   double begin, double end, Visit visit)
{
  if (!(begin < end)) return 0.;
  GridCell cell, step;
  GridPoint next, interval;
  for (int axis = 0; axis < 3; ++axis) {
    const double position = start[axis] + begin * delta[axis];
    if (!(position >= 0 && position < size[axis])) return 0.;
    const double lower = std::floor(position);
    cell[axis] = static_cast<int>(lower);
    step[axis] = (delta[axis] > 0) - (delta[axis] < 0);
    if (step[axis] < 0 && position == lower) --cell[axis];
    if (step[axis] == 0) {
      interval[axis] = next[axis] = std::numeric_limits<double>::infinity();
    } else {
      const double boundary = cell[axis] + (step[axis] > 0 ? 1 : 0);
      next[axis] = (boundary - start[axis]) / delta[axis];
      interval[axis] = std::abs(1.0 / delta[axis]);
    }
  }
  const double first = begin;
  while (cell[0] >= 0 && cell[0] < size[0] && cell[1] >= 0 && cell[1] < size[1] &&
         cell[2] >= 0 && cell[2] < size[2]) {
    const double crossing = std::min({next[0], next[1], next[2]});
    if (crossing > begin) visit(cell);
    if (crossing >= end - 1e-12) return end;
    for (int axis = 0; axis < 3; ++axis) {
      if (next[axis] <= crossing + 1e-12) {
        cell[axis] += step[axis];
        next[axis] += interval[axis];
      }
    }
    begin = crossing;
  }
  return begin > first ? begin : 0.;
}
}  // namespace obstacle_detection
