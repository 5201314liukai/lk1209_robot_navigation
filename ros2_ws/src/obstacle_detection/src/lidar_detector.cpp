
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarDetector : public rclcpp::Node
{
public:
    LidarDetector()
    : Node("lidar_detector")
    {
        // 订阅激光雷达数据
        scan_sub_ = this->create_subscription<
            sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &LidarDetector::scanCallback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "C++ LiDAR detector started"
        );
    }

private:

    // 雷达数据回调函数
    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received LiDAR scan: %zu points",
            msg->ranges.size()
        );
    }

    // ROS2 订阅对象
    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarDetector>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

