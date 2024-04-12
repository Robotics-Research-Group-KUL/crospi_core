message("Compiling node located in ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(etasl REQUIRED)
find_package(etasl_ros2 REQUIRED) #Needed to read srv and msgs generated with rosidl_generate_interfaces


add_executable(etasl_node etasl_node.cpp etasl_node.hpp)
# add_executable(etasl_node src/etasl_node.cpp)

target_link_libraries(etasl_node 
        etasl::expressiongraph_context 
        etasl::expressiongraph_solver_qpoases 
        etasl::expressiongraph_context_lua 
        etasl_misc_lib 
        etasl_task_utils
        etasl_node_utils)

# ament_target_dependencies(etasl_node rclcpp std_msgs sensor_msgs)

ament_target_dependencies(etasl_node
  "etasl_ros2" #Needed to read srv and msgs generated with rosidl_generate_interfaces
  "rclcpp"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
)


target_include_directories(etasl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(etasl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS etasl_node
  DESTINATION lib/${PROJECT_NAME})


  