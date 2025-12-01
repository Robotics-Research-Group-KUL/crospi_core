message("Compiling node located in ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(etasl REQUIRED)
# find_package(crospi_core REQUIRED) #Needed to read srv and msgs generated with rosidl_generate_interfaces


add_executable(erwin_node erwin_node_test.cpp)
# add_executable(crospi_node src/crospi_node.cpp)

target_link_libraries(erwin_node 
        etasl::expressiongraph_context 
        etasl::expressiongraph_solver_qpoases 
        etasl::expressiongraph_context_lua 
        etasl_misc_lib 
        crospi_utils
        crospi_node_utils)

# ament_target_dependencies(crospi_node rclcpp std_msgs sensor_msgs)

ament_target_dependencies(erwin_node
  # "crospi_core" #Needed to read srv and msgs generated with rosidl_generate_interfaces
  "rclcpp"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
)


target_include_directories(erwin_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(erwin_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS erwin_node
  DESTINATION lib/${PROJECT_NAME})


  