cmake_minimum_required(VERSION 3.8)
project(lk1209_astar_planner)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED src/grid.cpp src/planner.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME} geometry_msgs nav_msgs nav2_core nav2_costmap_2d
  pluginlib rclcpp rclcpp_lifecycle tf2_ros)
pluginlib_export_plugin_description_file(nav2_core planner_plugin.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid test/test_grid.cpp)
  target_link_libraries(test_grid ${PROJECT_NAME})
endif()

ament_package()
