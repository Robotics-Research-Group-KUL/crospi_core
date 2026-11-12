message("Compiling misc/crospi_node_utils libraries ${CMAKE_CURRENT_SOURCE_DIR}")

# Dependencies are found (and exported) in the top-level CMakeLists.txt.
# PUBLIC: packages linking crospi_core::crospi_node_utils inherit these dependencies.
add_library(crospi_node_utils SHARED
    io_handler_manager.cpp
    robot_driver_manager_lockfree.cpp
    multiple_drivers_manager.cpp
)

target_include_directories(crospi_node_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LUA_INCLUDE_DIR})

target_link_libraries(crospi_node_utils PUBLIC
  crospi_utils
  fmt::fmt
  jsoncpp
  Eigen3::Eigen
  etasl::expressiongraph_context
  etasl::expressiongraph_solver_qpoases
  etasl::expressiongraph_context_lua
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_action::rclcpp_action
  resource_retriever::resource_retriever
  ament_index_cpp::ament_index_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  ${crospi_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

install(
  TARGETS crospi_node_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
