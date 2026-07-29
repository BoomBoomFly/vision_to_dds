cmake_minimum_required(VERSION 3.5)
project(vision_to_dds)

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
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(vision_contract src/vision_contract.cpp)
target_include_directories(vision_contract PUBLIC include)
ament_target_dependencies(vision_contract tf2)

add_library(t265_health_adapter src/t265_health_adapter.cpp)
target_include_directories(t265_health_adapter PUBLIC include)

add_library(vision_to_dds_node_lib src/vision_to_dds.cpp)
target_compile_definitions(vision_to_dds_node_lib PRIVATE VISION_TO_DDS_NO_MAIN)
target_include_directories(vision_to_dds_node_lib PUBLIC include)
target_link_libraries(vision_to_dds_node_lib vision_contract)
ament_target_dependencies(
  vision_to_dds_node_lib
  rclcpp
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
  std_msgs
  std_srvs
)

add_executable(vision_to_dds_node src/vision_to_dds.cpp)
target_include_directories(vision_to_dds_node PUBLIC include)
target_link_libraries(vision_to_dds_node vision_contract)
ament_target_dependencies(
  vision_to_dds_node
  rclcpp
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
  std_msgs
  std_srvs
)

add_executable(t265_health_adapter_node src/t265_health_adapter_node.cpp)
target_include_directories(t265_health_adapter_node PUBLIC include)
target_link_libraries(t265_health_adapter_node t265_health_adapter)
ament_target_dependencies(t265_health_adapter_node rclcpp nav_msgs std_msgs)

install(TARGETS vision_contract t265_health_adapter vision_to_dds_node t265_health_adapter_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md VISION_DDS_H0.md T265_TF_CONTRACT.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vision_contract test/test_vision_contract.cpp)
  target_link_libraries(test_vision_contract vision_contract)
  ament_target_dependencies(test_vision_contract tf2)
  ament_add_gtest(test_t265_health_adapter test/test_t265_health_adapter.cpp)
  target_link_libraries(test_t265_health_adapter t265_health_adapter vision_contract)
  ament_target_dependencies(test_t265_health_adapter tf2)
  ament_add_gtest(test_enable_vision_dds test/test_enable_vision_dds.cpp)
  target_link_libraries(test_enable_vision_dds vision_to_dds_node_lib vision_contract)
  ament_target_dependencies(
    test_enable_vision_dds
    rclcpp
    builtin_interfaces
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    px4_msgs
    std_msgs
    std_srvs
  )
  ament_add_gtest(test_t265_tf_contract test/test_t265_tf_contract.cpp)
  target_link_libraries(test_t265_tf_contract vision_contract)
  ament_target_dependencies(test_t265_tf_contract tf2)
endif()

ament_package()
