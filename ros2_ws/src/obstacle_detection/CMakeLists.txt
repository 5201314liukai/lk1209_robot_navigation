cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

# ==========================================
# 1. C++ 编译标准
# ==========================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==========================================
# 2. 查找 ROS2 依赖
# ==========================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lk1209_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# ==========================================
# 3. 创建 C++ 可执行程序
# ==========================================
add_library(lidar_processing STATIC
  src/scan_processor.cpp
  src/obstacle_grouper.cpp
  src/segment_fitter.cpp
  src/segment_extractor.cpp
  src/segment_merger.cpp
  src/obstacle_extractor.cpp
)
target_include_directories(lidar_processing PUBLIC include)
ament_target_dependencies(lidar_processing sensor_msgs)

add_executable(lidar_detector src/nodes/lidar_detector_node.cpp)
target_link_libraries(lidar_detector lidar_processing)
add_executable(cloud_obstacles src/nodes/cloud_obstacles.cpp)
add_library(visibility_voxel_layer SHARED src/visibility_voxel_layer.cpp)
target_include_directories(visibility_voxel_layer PRIVATE include)
ament_target_dependencies(visibility_voxel_layer nav2_costmap_2d pluginlib sensor_msgs)
install(TARGETS visibility_voxel_layer LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(nav2_costmap_2d visibility_voxel_plugin.xml)
target_include_directories(cloud_obstacles PRIVATE include)
ament_target_dependencies(cloud_obstacles rclcpp sensor_msgs tf2_ros tf2_geometry_msgs)

# ==========================================
# 4. 设置头文件目录
# ==========================================

target_include_directories(lidar_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==========================================
# 5. 关联 ROS2 依赖
# ==========================================

ament_target_dependencies(lidar_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  lk1209_interfaces
)

# ==========================================
# 6. 安装可执行程序
# ==========================================

install(TARGETS
  lidar_detector
  cloud_obstacles
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# ==========================================
# 7. 测试配置
# ==========================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_segments test/test_scan_segments.cpp)
  target_link_libraries(test_scan_segments lidar_processing)
  ament_add_gtest(test_cloud_filter test/test_cloud_filter.cpp)
  target_include_directories(test_cloud_filter PRIVATE include)
  ament_add_gtest(test_visible_ray test/test_visible_ray.cpp)
  target_include_directories(test_visible_ray PRIVATE include)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# ==========================================
# 8. ROS2 功能包导出
# ==========================================


add_executable(test_segment_fitter
  test/test_segment_fitter.cpp
  src/segment_fitter.cpp
)

target_include_directories(test_segment_fitter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  test_segment_fitter
  DESTINATION lib/${PROJECT_NAME}
)


# Split and Merge 独立测试程序
add_executable(test_segment_extractor
  test/test_segment_extractor.cpp
  src/segment_fitter.cpp
  src/segment_extractor.cpp
  src/segment_merger.cpp
  src/obstacle_extractor.cpp
)

target_include_directories(test_segment_extractor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  test_segment_extractor
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_segment_merger
  test/test_segment_merger.cpp
  src/segment_merger.cpp
)

target_include_directories(test_segment_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  test_segment_merger
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(test_obstacle_extractor
  test/test_obstacle_extractor.cpp
  src/segment_fitter.cpp
  src/segment_extractor.cpp
  src/segment_merger.cpp
  src/obstacle_extractor.cpp
)

target_include_directories(test_obstacle_extractor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  test_obstacle_extractor
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

