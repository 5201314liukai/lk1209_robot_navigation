#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "lk1209_interfaces/msg/obstacle_segments.hpp"
#include "obstacle_detection/scan_processor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;
using ObstacleSegments = lk1209_interfaces::msg::ObstacleSegments;

namespace
{
geometry_msgs::msg::Point toMessage(const obstacle_detection::Point & p)
{
  geometry_msgs::msg::Point result;
  result.x = p.x;
  result.y = p.y;
  return result;
}
}  // namespace

class LidarDetector : public rclcpp::Node
{
public:
  LidarDetector() : Node("lidar_detector")
  {
    publisher_ = create_publisher<ObstacleSegments>("obstacle_segments", rclcpp::SensorDataQoS());
    points_ = create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_points", rclcpp::SensorDataQoS());
    subscription_ = create_subscription<LaserScan>(
      "scan", rclcpp::SensorDataQoS(), [this](LaserScan::ConstSharedPtr scan) {
        ObstacleSegments msg;
        msg.header = scan->header;
        auto features = processor_.features(*scan);
        msg.starts.reserve(features.segments.size());
        msg.ends.reserve(features.segments.size());
        for (const auto & segment : features.segments) {
          msg.starts.push_back(toMessage(segment.first_point));
          msg.ends.push_back(toMessage(segment.last_point));
        }
        publisher_->publish(msg);
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = scan->header;
        cloud.height = 1;
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(features.points.size());
        if (!features.points.empty()) {
          sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
          for (const auto & point : features.points) {
            *x = static_cast<float>(point.x);
            *y = static_cast<float>(point.y);
            *z = 0.0F;
            ++x;
            ++y;
            ++z;
          }
        }
        points_->publish(cloud);
      });
    RCLCPP_INFO(get_logger(), "Publishing line segments in the input laser frame");
  }

private:
  obstacle_detection::ScanProcessor processor_;
  rclcpp::Publisher<ObstacleSegments>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_;
  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDetector>());
  rclcpp::shutdown();
  return 0;
}
