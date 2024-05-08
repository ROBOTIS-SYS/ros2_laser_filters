cmake_minimum_required(VERSION 3.5)
project(ros2_laser_filters)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)


add_executable(scan_filter_chain
  src/scan_filter_chain.cpp
  src/box_filter.cpp
  src/radius_search_filter.cpp
  src/pose_filter.cpp
)
target_include_directories(scan_filter_chain PRIVATE
  "include"
)
ament_target_dependencies(scan_filter_chain
  angles
  laser_geometry
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS
  scan_filter_chain
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(DYNAMIC_TESTING)

  # viwer vtk dependencies
  # find_package(VTK 9.0 REQUIRED)
  # add_library(VTK_VIEWER_LIB)
  # target_link_libraries(VTK_VIEWER_LIB
  # ${VTK_LIBRARIES})

  # target_include_directories(VTK_VIEWER_3D_LIB PUBLIC
  # ${CMAKE_SOURCE_DIR}/include
  # ${CMAKE_SOURCE_DIR}/src)

  add_executable(test_pose_predictor test/test_pose_predictor.cpp src/pose_filter.cpp)
  # target_link_libraries(test_pose_predictor PUBLIC VTK_VIEWER_3D_LIB)
  # vtk_module_autoinit(TARGETS test_pose_predictor MODULES ${VTK_LIBRARIES} )
  target_compile_options(test_pose_predictor PRIVATE -g)
  # scan_filter_chain
  include_directories(test_pose_predictor PRIVATE
  "include"
  )

  ament_target_dependencies(test_pose_predictor
  angles
  laser_geometry
  rclcpp
  sensor_msgs
  tf2_ros
  )
endif()

ament_package()
