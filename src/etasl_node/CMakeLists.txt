message("Compiling node located in ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(etasl REQUIRED)


add_executable(etasl_node etasl_node.cpp etasl_node.hpp featurevariableinitializer.cpp IO_handlers.cpp )
# add_executable(etasl_node src/etasl_node.cpp)

target_link_libraries(etasl_node etasl::expressiongraph_context etasl::expressiongraph_solver_qpoases etasl::expressiongraph_context_lua )

ament_target_dependencies(etasl_node rclcpp std_msgs sensor_msgs)


target_include_directories(etasl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(etasl_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS etasl_node
  DESTINATION lib/${PROJECT_NAME})


  