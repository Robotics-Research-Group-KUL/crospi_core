message("Compiling node located in ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(etasl REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(etasl_ros2 REQUIRED) #Needed to read srv and msgs generated with rosidl_generate_interfaces

# -- start --Stuff for triple buffer implementation of boost 1.89
set(BOOST_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local/lib/boost/1.89.0" ${CMAKE_PREFIX_PATH})
find_package(Boost 1.89 REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")

# -- end -- Stuff for triple buffer implementation of boost 1.89

include_directories(.)

add_executable(etasl_node etasl_node.cpp)

# add_executable(etasl_node src/etasl_node.cpp)

target_link_libraries(etasl_node 
        etasl::expressiongraph_context 
        etasl::expressiongraph_solver_qpoases 
        etasl::expressiongraph_context_lua 
        etasl_misc_lib 
        etasl_task_utils
        etasl_node_utils
        robot_interfacing_utils
        # buffer_utils
        # triple_buffer
        # kuka_iiwa_driver
        # simple_kinematic_simulation
        )

# ament_target_dependencies(etasl_node rclcpp std_msgs sensor_msgs)

ament_target_dependencies(etasl_node
  # "etasl_ros2" #Needed to read srv and msgs generated with rosidl_generate_interfaces
  "rclcpp"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "pluginlib"
)


# TODO: change the directories of  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> and $<INSTALL_INTERFACE:include>
target_include_directories(etasl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})

target_compile_features(etasl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS etasl_node
  DESTINATION lib/${PROJECT_NAME})


  