#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "obstacle_detection/cloud_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;
using Scan = sensor_msgs::msg::LaserScan;
using Field = sensor_msgs::msg::PointField;
using namespace obstacle_detection::cloud;
constexpr unsigned kColumns = 360, kRings = 16, kHorizontalRing = 10;
constexpr double kPi = 3.141592653589793;
constexpr double kMinRange = 0.12, kMaxRange = 8.0;

std::optional<std::size_t> float_offset(const Cloud &cloud, const char *name)
{
  for (const auto &field : cloud.fields) {
    if (field.name == name && field.datatype == Field::FLOAT32 && field.count == 1 &&
        std::size_t(field.offset) + sizeof(float) <= cloud.point_step) return field.offset;
  }
  return std::nullopt;
}

float read_float(const std::uint8_t *p, std::size_t offset)
{
  float value;
  std::memcpy(&value, p + offset, sizeof(value));
  return value;
}

Cloud xyz_cloud(const std_msgs::msg::Header &header, const std::vector<Point> &points)
{
  Cloud result;
  result.header = header;
  result.height = 1;
  result.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(result);
  modifier.setPointCloud2Fields(3, "x", 1, Field::FLOAT32, "y", 1, Field::FLOAT32,
    "z", 1, Field::FLOAT32);
  modifier.resize(points.size());
  if (!points.empty()) {
    sensor_msgs::PointCloud2Iterator<float> x(result, "x"), y(result, "y"), z(result, "z");
    for (const auto &p : points) {
      *x = static_cast<float>(p.x); *y = static_cast<float>(p.y); *z = static_cast<float>(p.z);
      ++x; ++y; ++z;
    }
  }
  return result;
}

class CloudObstacles : public rclcpp::Node
{
public:
  CloudObstacles() : Node("cloud_obstacles"), buffer_(get_clock()), listener_(buffer_)
  {
    // Matches the 16-ring Gazebo profile (inclusive endpoints on both axes).
    for (unsigned row = 0; row < kRings; ++row) {
      const double elevation = (-20.0 + 2.0*row)*kPi/180.0;
      for (unsigned col = 0; col < kColumns; ++col) {
        const double azimuth = -kPi + col*2*kPi/(kColumns-1);
        beams_[row*kColumns+col] = {std::cos(elevation)*std::cos(azimuth),
          std::cos(elevation)*std::sin(azimuth), std::sin(elevation)};
        floor_rays_[row*kColumns+col] = ogre2_floor_ray(beams_[row*kColumns+col]);
      }
    }
    scans_ = create_publisher<Scan>("scan", 2);
    obstacles_ = create_publisher<Cloud>("lidar/obstacles", 2);
    clearing_ = create_publisher<Cloud>("lidar/clearing", 2);
    input_ = create_subscription<Cloud>("lidar/points", rclcpp::SensorDataQoS().keep_last(2),
      [this](Cloud::ConstSharedPtr cloud) {process(*cloud);});
  }

private:
  void process(const Cloud &cloud)
  {
    const auto x = float_offset(cloud, "x"), y = float_offset(cloud, "y"), z = float_offset(cloud, "z");
    if (cloud.width != kColumns || cloud.height != kRings || cloud.is_bigendian ||
        cloud.header.frame_id != "lidar_link" || !x || !y || !z ||
        std::size_t(cloud.point_step)*cloud.width > cloud.row_step ||
        std::size_t(cloud.row_step)*cloud.height > cloud.data.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Expected organized little-endian %ux%u float XYZ in lidar_link; received %ux%u in %s",
        kColumns, kRings, cloud.width, cloud.height, cloud.header.frame_id.c_str());
      return;
    }
    tf2::Transform to_base;
    try {
      auto tf = buffer_.lookupTransform("base_footprint", cloud.header.frame_id, rclcpp::Time(cloud.header.stamp));
      tf2::fromMsg(tf.transform, to_base);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Cloud transform unavailable: %s", ex.what());
      return;
    }
    Scan scan;
    scan.header = cloud.header;
    scan.angle_min = -kPi; scan.angle_max = kPi; scan.angle_increment = 2*kPi/(kColumns-1);
    scan.range_min = kMinRange; scan.range_max = kMaxRange; scan.scan_time = 0.10;
    scan.ranges.assign(kColumns, std::numeric_limits<float>::quiet_NaN());
    struct Hit { Point sensor, base, floor_test; };
    std::vector<Hit> body_height_hits;
    std::vector<Point> floor_candidates, endpoints, obstacles;
    const bool send_endpoints = clearing_->get_subscription_count() != 0;
    if (send_endpoints) endpoints.reserve(beams_.size());
    for (unsigned row = 0; row < kRings; ++row) {
      for (unsigned col = 0; col < kColumns; ++col) {
        const auto *data = cloud.data.data() + std::size_t(row)*cloud.row_step +
          std::size_t(col)*cloud.point_step;
        const Point p{read_float(data,*x), read_float(data,*y), read_float(data,*z)};
        const auto &beam = beams_[row*kColumns+col];
        const double range = range_of(p,beam);
        if (std::isinf(range)) {
          if (row == kHorizontalRing) scan.ranges[col] = static_cast<float>(range);
          if (range > 0 && send_endpoints)
            endpoints.push_back({kMaxRange*beam.x, kMaxRange*beam.y, kMaxRange*beam.z});
          continue;
        }
        if (!std::isfinite(range) || range < kMinRange || range > kMaxRange) continue;
        const auto v = to_base * tf2::Vector3(p.x,p.y,p.z);
        const Point base{v.x(),v.y(),v.z()};
        // A self return occludes the scene. Leave its scan bin unknown and
        // emit neither a marking point nor a ray that clears beyond the robot.
        if (on_chassis_top(base)) continue;
        if (row == kHorizontalRing) scan.ranges[col] = static_cast<float>(std::hypot(p.x,p.y));
        if (send_endpoints) endpoints.push_back(p);
        // Base footprint is at floor level. The highest lidar collision surface
        // is 0.20 m; keep a 2 cm clearance margin above it.
        Point floor_test = base;
        if (row < kHorizontalRing) {
          const auto &ray = floor_rays_[row*kColumns+col];
          const auto sampled = to_base * tf2::Vector3(range*ray.x, range*ray.y, range*ray.z);
          floor_test = {sampled.x(), sampled.y(), sampled.z()};
        }
        if (floor_test.z < -0.035 || floor_test.z > 0.22) continue;
        body_height_hits.push_back({p, base, floor_test});
        if (std::abs(floor_test.z) <= 0.035) floor_candidates.push_back(floor_test);
      }
    }

    const auto floor = fit_ground(floor_candidates);
    std::unordered_set<std::uint64_t> occupied;
    for (const auto &hit : body_height_hits) {
      if (floor && floor->contains(hit.floor_test)) continue;
      // Keep a measured point rather than an averaged point between surfaces.
      if (occupied.insert(voxel_key(hit.base)).second) obstacles.push_back(hit.sensor);
    }
    scans_->publish(scan);
    if (send_endpoints) clearing_->publish(xyz_cloud(cloud.header,endpoints));
    obstacles_->publish(xyz_cloud(cloud.header,obstacles));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Cloud: ground model %s (%zu candidates), %zu obstacle voxels",
      floor ? "supported" : "unavailable", floor_candidates.size(), obstacles.size());
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::array<Point,kColumns*kRings> beams_;
  std::array<Point,kColumns*kRings> floor_rays_;
  rclcpp::Publisher<Scan>::SharedPtr scans_;
  rclcpp::Publisher<Cloud>::SharedPtr obstacles_, clearing_;
  rclcpp::Subscription<Cloud>::SharedPtr input_;
};
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<CloudObstacles>());
  rclcpp::shutdown();
  return 0;
}
