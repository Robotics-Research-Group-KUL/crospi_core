message("Compiling node located in ${CMAKE_CURRENT_SOURCE_DIR}")


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(simple_robot_node simple_robot_node.cpp)
# add_executable(simple_robot_node src/simple_robot_node.cpp)

ament_target_dependencies(simple_robot_node rclcpp std_msgs sensor_msgs)


target_include_directories(simple_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_robot_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS simple_robot_node
  DESTINATION lib/${PROJECT_NAME})
