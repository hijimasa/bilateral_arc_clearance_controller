# bilateral_arc_clearance_controller: arc-clearance obstacle avoidance.
#
# Layout follows nav2 controller plugin conventions. Builds in two modes:
#  - ament/colcon (ROS 2 workspace): core library + cmd_vel filter node +
#    nav2 controller plugin.
#  - Plain CMake (add_subdirectory or -S .): core library + scenario harness
#    only (no ROS dependency).
cmake_minimum_required(VERSION 3.16)
project(bilateral_arc_clearance_controller VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Framework-free algorithm core
add_library(bac_core STATIC src/bac_core.cpp)
target_include_directories(bac_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(bac_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(bac_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)
  install(FILES README.md README.ja.md LICENSE CHANGELOG.rst DESTINATION share/${PROJECT_NAME})
  install(TARGETS bac_core
      EXPORT export_bac_core
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
  )
  ament_export_targets(export_bac_core HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)

  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(nav2_core REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  find_package(nav2_util REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  # Evaluation node: cmd_vel filter driven by a laser scan
  add_executable(bac_filter_node src/bac_filter_node.cpp)
  target_link_libraries(bac_filter_node bac_core)
  target_compile_options(bac_filter_node PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(bac_filter_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
  install(TARGETS bac_filter_node DESTINATION lib/${PROJECT_NAME})

  # nav2 controller plugin (verified against the ROS 2 Jazzy nav2_core API)
  add_library(bilateral_arc_clearance_controller SHARED src/bac_controller.cpp)
  target_link_libraries(bilateral_arc_clearance_controller bac_core)
  target_compile_options(bilateral_arc_clearance_controller PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(bilateral_arc_clearance_controller
      rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_costmap_2d nav2_util
      geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs
  )
  install(TARGETS bilateral_arc_clearance_controller
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
  )
  pluginlib_export_plugin_description_file(nav2_core bilateral_arc_clearance_controller_plugin.xml)
endif()

# Closed-loop scenario harness (framework-free; also the package's test)
include(CTest)
if(BUILD_TESTING)
  add_executable(bac_core_unit test/core_unit.cpp)
  target_link_libraries(bac_core_unit bac_core)
  target_compile_options(bac_core_unit PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacCoreUnit COMMAND bac_core_unit)

  add_executable(bac_perf_benchmark test/perf_benchmark.cpp)
  target_link_libraries(bac_perf_benchmark bac_core)
  target_compile_options(bac_perf_benchmark PRIVATE -Wall -Wextra -Wpedantic)

  add_executable(bac_scenario_harness test/scenarios.cpp)
  target_include_directories(bac_scenario_harness PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  target_link_libraries(bac_scenario_harness bac_core)
  target_compile_options(bac_scenario_harness PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME BacScenarioHarness COMMAND bac_scenario_harness --strict)
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
