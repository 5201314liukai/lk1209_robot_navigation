cmake_minimum_required(VERSION 3.8)
project(robot_description)

# ROS2 构建工具
find_package(ament_cmake REQUIRED)

# 安装机器人模型和启动文件
install(
  DIRECTORY
    urdf
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

