cmake_minimum_required(VERSION 3.8)
project(map_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(map_generator_component SHARED
  src/map_generator_component.cpp
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

target_compile_definitions(map_generator_component
  PRIVATE "MAP_GENERATOR_MAP_GENERATOR_COMPONENT_BUILDING_DLL"
)

rclcpp_components_register_nodes(map_generator_component
  "map_generator::MapGeneratorComponent"
)

ament_auto_add_executable(map_generator_node
  src/map_generator_node.cpp)

target_link_libraries(map_generator_node
  map_generator_component
)

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
