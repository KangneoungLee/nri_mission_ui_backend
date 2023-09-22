cmake_minimum_required(VERSION 3.5)
project(nri_plan_partition)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  {Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(executable_name nri_plan_partition_node)


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_library(voro_cell src/voro_cell.cpp)
target_link_libraries(voro_cell ${Boost_LIBRARIES})

add_library(dynamic_voronoi_core SHARED
  src/dynamic_voronoi.cpp
)
target_link_libraries(dynamic_voronoi_core voro_cell ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(${executable_name}
  src/nri_plan_partition.cpp
)

target_link_libraries(${executable_name} dynamic_voronoi_core ${OpenCV_LIBS})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
#ament_export_dependencies(rosidl_default_runtime)
ament_package()
