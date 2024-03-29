cmake_minimum_required(VERSION 3.8)
project(ouagv_2d_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_auto_find_build_dependencies()

ament_auto_add_library(twod_slam_component SHARED
  src/twod_slam_component.cpp
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# -------------------------------------------------------------
target_compile_definitions(twod_slam_component
  PRIVATE "TWOD_SLAM_TWOD_SLAM_COMPONENT_BUILDING_DLL"
)

rclcpp_components_register_nodes(twod_slam_component
  "twod_slam::TwodSlamComponent"
)

ament_auto_add_executable(twod_slam_node
  src/twod_slam_node.cpp
  src/impl/pointcloud_manager.cpp
  src/impl/map_manager.cpp
  src/impl/icp_scan_matching.cpp
)

target_link_libraries(twod_slam_node
  twod_slam_component
  Eigen3::Eigen
)

# -------------------------------------------------------------

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
