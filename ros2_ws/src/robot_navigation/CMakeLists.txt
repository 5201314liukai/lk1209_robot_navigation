cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_module(launch/nav_profiles.py DESTINATION_SUFFIX lk1209_navigation)

install(
  DIRECTORY
    config
    launch
    maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/house_delivery scripts/delivery_dispatcher
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/delivery_routing.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(delivery_routing test/test_delivery_routing.py)
  ament_add_pytest_test(nav_profiles test/test_nav_profiles.py)
endif()

ament_package()

