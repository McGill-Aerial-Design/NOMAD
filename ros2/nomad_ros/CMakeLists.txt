cmake_minimum_required(VERSION 3.8)
project(nomad_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# The package consumes the repository's receive-only MAVLink observer target.
# Exclude unrelated command-capable core targets from the ROS image build.
set(BUILD_TESTING OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Repository root: ros2/nomad_ros -> NOMAD/.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. nomad_core_build EXCLUDE_FROM_ALL)
unset(BUILD_TESTING)

# Pure telemetry-to-ROS translation, kept separate from node wiring.
add_library(nomad_ros_translation
  src/translation.cpp
)
target_include_directories(nomad_ros_translation PUBLIC include)
# Plain (non-keyword) form: ament_target_dependencies also uses it internally.
target_link_libraries(nomad_ros_translation nomad_mavlink_observation)
ament_target_dependencies(nomad_ros_translation
  sensor_msgs
)

add_executable(nomad_vehicle_node
  src/node.cpp
)
target_include_directories(nomad_vehicle_node PRIVATE include)
# The ROS node receives only the restricted MAVLink telemetry interface. It does
# not link nomad_core or the command-capable MavlinkConnection target.
target_link_libraries(nomad_vehicle_node
  nomad_ros_translation
  nomad_mavlink_observation
)
ament_target_dependencies(nomad_vehicle_node
  rclcpp
  std_msgs
  sensor_msgs
)

option(NOMAD_ROS_BUILD_TESTS "Build the nomad_ros translation unit tests" ON)
if(NOMAD_ROS_BUILD_TESTS)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(nomad_ros_translation_tests tests/translation_test.cpp)
  if(TARGET nomad_ros_translation_tests)
    target_include_directories(nomad_ros_translation_tests PRIVATE include)
    target_link_libraries(nomad_ros_translation_tests nomad_ros_translation nomad_mavlink_observation)
    ament_target_dependencies(nomad_ros_translation_tests
      sensor_msgs
    )
  endif()

  ament_add_gtest(nomad_ros_observation_contract_tests tests/observation_contract_test.cpp)
  if(TARGET nomad_ros_observation_contract_tests)
    target_link_libraries(nomad_ros_observation_contract_tests nomad_mavlink_observation)
  endif()
endif()

install(TARGETS
  nomad_vehicle_node
  nomad_ros_translation
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
