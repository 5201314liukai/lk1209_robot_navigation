#include <algorithm>
#include <cmath>
#include <vector>
#include "nav2_costmap_2d/voxel_layer.hpp"
#include "obstacle_detection/visible_ray.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace obstacle_detection
{
class VisibilityVoxelLayer : public nav2_costmap_2d::VoxelLayer
{
protected:
  void raytraceFreespace(const nav2_costmap_2d::Observation &obs,
                         double *min_x, double *min_y, double *max_x, double *max_y) override
  {
    const auto &cloud = *obs.cloud_;
    if (!cloud.width || !cloud.height) return;
    GridPoint origin;
    const auto &o = obs.origin_;
    if (!worldToMap3DFloat(o.x, o.y, o.z, origin[0], origin[1], origin[2])) return;
    const GridCell size{static_cast<int>(size_x_), static_cast<int>(size_y_), size_z_};
    const bool trace_endpoints = clearing_endpoints_pub_->get_subscription_count() != 0;
    std::vector<GridPoint> endpoints;
    int low_x = size[0], low_y = size[1], high_x = -1, high_y = -1;
    auto clear = [&](const GridCell &cell) {
      const unsigned x = cell[0], y = cell[1], z = cell[2];
      const unsigned index = getIndex(x, y);
      if (!(voxel_grid_.getData()[index] & (0x10001u << z))) return;
      voxel_grid_.clearVoxel(x, y, z);
      const auto status = voxel_grid_.getVoxelColumn(x, y, unknown_threshold_, mark_threshold_);
      if (status == nav2_voxel_grid::MARKED) return;
      const auto cost = status == nav2_voxel_grid::FREE ?
        nav2_costmap_2d::FREE_SPACE : nav2_costmap_2d::NO_INFORMATION;
      if (costmap_[index] == cost) return;
      costmap_[index] = cost;
      low_x = std::min(low_x, cell[0]);
      low_y = std::min(low_y, cell[1]);
      high_x = std::max(high_x, cell[0]);
      high_y = std::max(high_y, cell[1]);
    };
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      const double dx = *x-o.x, dy = *y-o.y, dz = *z-o.z;
      const double range = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (!std::isfinite(range) || range <= 0.) continue;
      // Preserve Humble's two-cell margin before each measured endpoint.
      const double stop = std::min(obs.raytrace_max_range_, range - 2 * resolution_);
      if (stop <= obs.raytrace_min_range_) continue;
      const GridPoint delta{dx/resolution_, dy/resolution_, dz/z_resolution_};
      const double end = visible_ray(origin, delta, size, obs.raytrace_min_range_/range,
                                     stop/range, clear);
      if (trace_endpoints && end > 0.) {
        endpoints.push_back({o.x+end*dx, o.y+end*dy, o.z+end*dz});
      }
    }
    if (high_x >= 0) {
      double wx, wy;
      mapToWorld(low_x, low_y, wx, wy);
      touch(wx, wy, min_x, min_y, max_x, max_y);
      mapToWorld(high_x, high_y, wx, wy);
      touch(wx, wy, min_x, min_y, max_x, max_y);
    }
    if (trace_endpoints) {
      sensor_msgs::msg::PointCloud2 msg;
      msg.header = cloud.header;
      msg.header.frame_id = global_frame_;
      msg.height = 1;
      sensor_msgs::PointCloud2Modifier fields(msg);
      fields.setPointCloud2FieldsByString(1, "xyz");
      fields.resize(endpoints.size());
      if (!endpoints.empty()) {
        sensor_msgs::PointCloud2Iterator<float> px(msg, "x"), py(msg, "y"), pz(msg, "z");
        for (const auto &p : endpoints) {
          *px = p[0]; *py = p[1]; *pz = p[2];
          ++px; ++py; ++pz;
        }
      }
      clearing_endpoints_pub_->publish(msg);
    }
  }
};
}  // namespace obstacle_detection

PLUGINLIB_EXPORT_CLASS(obstacle_detection::VisibilityVoxelLayer, nav2_costmap_2d::Layer)
