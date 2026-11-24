#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "lk1209_astar_planner/grid.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace lk1209_astar_planner
{
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Costmap = nav2_costmap_2d::Costmap2D;
static_assert(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE == 253, "Unexpected cost encoding");

class FourNeighborAStar : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    if (!node || !costmap_ros || !costmap_ros->getCostmap()) {
      throw nav2_core::PlannerException("FourNeighborAStar requires a node and a costmap");
    }
    const auto parameter = name + ".cost_weight";
    if (!node->has_parameter(parameter)) {
      node->declare_parameter<double>(parameter, 4.0);
    }
    cost_weight_ = node->get_parameter(parameter).as_double();
    if (!std::isfinite(cost_weight_) || cost_weight_ < 0.0) {
      throw nav2_core::PlannerException(parameter + " must be finite and nonnegative");
    }
    costmap_ros_ = costmap_ros;
    clock_ = node->get_clock();
  }

  void activate() override {}
  void deactivate() override {}
  void cleanup() override
  {
    costmap_ros_.reset();
    clock_.reset();
  }
  nav_msgs::msg::Path createPlan(const PoseStamped & start, const PoseStamped & goal) override;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Clock::SharedPtr clock_;
  double cost_weight_ = 4.0;
};

static std::size_t cellForPose(const Costmap & grid, const PoseStamped & pose)
{
  const double x = pose.pose.position.x, y = pose.pose.position.y;
  unsigned int mx, my;
  // Check upper bounds before worldToMap casts floating point coordinates to unsigned.
  if (!std::isfinite(x) || !std::isfinite(y) ||
    x >= grid.getOriginX() + grid.getSizeInCellsX() * grid.getResolution() ||
    y >= grid.getOriginY() + grid.getSizeInCellsY() * grid.getResolution() ||
    !grid.worldToMap(x, y, mx, my))
  {
    throw nav2_core::PlannerException("A path endpoint is outside the global costmap");
  }
  if (grid.getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    throw nav2_core::PlannerException("A path endpoint is occupied or unknown");
  }
  return static_cast<std::size_t>(my) * grid.getSizeInCellsX() + mx;
}

nav_msgs::msg::Path FourNeighborAStar::createPlan(
  const PoseStamped & start, const PoseStamped & goal)
{
  if (!costmap_ros_ || !clock_) {
    throw nav2_core::PlannerException("FourNeighborAStar is not configured");
  }
  const auto frame = costmap_ros_->getGlobalFrameID();
  if (start.header.frame_id != frame || goal.header.frame_id != frame) {
    throw nav2_core::PlannerException("Both endpoints must use the global costmap frame " + frame);
  }
  Costmap snapshot;
  auto * live = costmap_ros_->getCostmap();
  {
    std::lock_guard<Costmap::mutex_t> guard(*live->getMutex());
    if (!live->getCharMap() || !live->getSizeInCellsX() || !live->getSizeInCellsY()) {
      throw nav2_core::PlannerException("Global costmap is empty");
    }
    snapshot = *live;
  }
  const auto width = snapshot.getSizeInCellsX();
  const auto route = findPath(
    snapshot.getCharMap(), width, snapshot.getSizeInCellsY(),
    cellForPose(snapshot, start), cellForPose(snapshot, goal), cost_weight_);
  if (route.empty()) {
    throw nav2_core::PlannerException("No four-neighbor path through the observed free space");
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = clock_->now();
  auto last = goal;
  last.header = path.header;
  if (start.pose.position.x == goal.pose.position.x && start.pose.position.y == goal.pose.position.y) {
    path.poses.push_back(last);
    return path;
  }
  auto first = start;
  first.header = path.header;
  path.poses.push_back(first);
  for (std::size_t i = 1; i + 1 < route.size(); ++i) {
    PoseStamped pose;
    pose.header = path.header;
    snapshot.mapToWorld(
      route[i] % width, route[i] / width, pose.pose.position.x, pose.pose.position.y);
    path.poses.push_back(pose);
  }
  path.poses.push_back(last);
  // Orient intermediate poses along the path; retain the requested terminal heading.
  for (std::size_t i = 1; i + 1 < path.poses.size(); ++i) {
    auto & p = path.poses[i].pose;
    const auto & next = path.poses[i + 1].pose.position;
    const double half_yaw = 0.5 * std::atan2(next.y - p.position.y, next.x - p.position.x);
    p.orientation.z = std::sin(half_yaw);
    p.orientation.w = std::cos(half_yaw);
  }
  return path;
}
}  // namespace lk1209_astar_planner

PLUGINLIB_EXPORT_CLASS(lk1209_astar_planner::FourNeighborAStar, nav2_core::GlobalPlanner)
