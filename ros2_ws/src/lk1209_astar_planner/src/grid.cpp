#include "lk1209_astar_planner/grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace lk1209_astar_planner
{
namespace
{
struct Candidate
{
  std::size_t index;
  double travel_cost;
  double estimated_total;
};

struct LowestCostFirst
{
  bool operator()(const Candidate & a, const Candidate & b) const
  {
    if (a.estimated_total != b.estimated_total) {
      return a.estimated_total > b.estimated_total;
    }
    if (a.travel_cost != b.travel_cost) {
      return a.travel_cost < b.travel_cost;
    }
    return a.index > b.index;
  }
};
}  // namespace

std::vector<std::size_t> findPath(
  const unsigned char * costs, std::size_t width, std::size_t height,
  std::size_t start, std::size_t goal, double cost_weight)
{
  constexpr unsigned char first_blocked = 253;  // inscribed, lethal, unknown
  if (!costs || width == 0 || height > std::numeric_limits<std::size_t>::max() / width ||
    !std::isfinite(cost_weight) || cost_weight < 0.0)
  {
    return {};
  }
  const auto count = width * height;
  if (start >= count || goal >= count || costs[start] >= first_blocked ||
    costs[goal] >= first_blocked)
  {
    return {};
  }
  const auto manhattan = [width, goal](std::size_t index) {
      const auto x = index % width, y = index / width;
      const auto gx = goal % width, gy = goal / width;
      return static_cast<double>(std::max(x, gx) - std::min(x, gx)) +
             static_cast<double>(std::max(y, gy) - std::min(y, gy));
    };
  std::priority_queue<Candidate, std::vector<Candidate>, LowestCostFirst> frontier;
  std::vector<double> best_cost(count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(count, count);
  frontier.push({start, 0.0, manhattan(start)});
  best_cost[start] = 0.0;
  while (!frontier.empty()) {
    const auto current = frontier.top();
    frontier.pop();
    if (current.travel_cost != best_cost[current.index]) {
      continue;  // A cheaper route to this cell was enqueued later.
    }
    if (current.index == goal) {
      std::vector<std::size_t> path;
      for (auto i = goal; i != count; i = parent[i]) {
        path.push_back(i);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    const auto visit = [&](std::size_t next) {
        if (costs[next] >= first_blocked) {
          return;
        }
        const double step = 1.0 + cost_weight * (costs[next] / 252.0);
        const double new_cost = current.travel_cost + step;
        if (new_cost < best_cost[next]) {
          best_cost[next] = new_cost;
          parent[next] = current.index;
          frontier.push({next, new_cost, new_cost + manhattan(next)});
        }
      };
    const auto x = current.index % width;
    const auto y = current.index / width;
    if (x > 0) {visit(current.index - 1);}
    if (x + 1 < width) {visit(current.index + 1);}
    if (y > 0) {visit(current.index - width);}
    if (y + 1 < height) {visit(current.index + width);}
  }
  return {};
}
}  // namespace lk1209_astar_planner
