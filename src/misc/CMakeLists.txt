message("Compiling misc libraries ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(etasl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_library(etasl_misc_lib 
            featurevariableinitializer.cpp 
            IO_handlers_deleteme.cpp 
            port_observer.cpp )

target_link_libraries(etasl_misc_lib 
                      etasl::expressiongraph_context 
                      etasl::expressiongraph_solver_qpoases 
                      etasl::expressiongraph_context_lua )


ament_target_dependencies(etasl_misc_lib
                          "rclcpp"
                          "std_msgs"
                          "sensor_msgs"
                        )

target_include_directories(etasl_misc_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(etasl_misc_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS etasl_misc_lib
  DESTINATION lib/${PROJECT_NAME})

add_subdirectory(etasl_node_utils)
add_subdirectory(etasl_task_utils)
add_subdirectory(robot_interfacing_utils)
# add_subdirectory(simple_kinematic_simulation)


