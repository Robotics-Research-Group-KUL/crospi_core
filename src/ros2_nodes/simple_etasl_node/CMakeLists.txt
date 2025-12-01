message("Compiling node located in ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(etasl REQUIRED)


add_executable(simple_crospi_node simple_crospi_node.cpp simple_crospi_node.hpp)
# add_executable(simple_crospi_node src/simple_crospi_node.cpp)

target_link_libraries(simple_crospi_node etasl::expressiongraph_context etasl::expressiongraph_solver_qpoases etasl::expressiongraph_context_lua etasl_misc_lib )

ament_target_dependencies(simple_crospi_node rclcpp std_msgs sensor_msgs)


target_include_directories(simple_crospi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_crospi_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS simple_crospi_node
  DESTINATION lib/${PROJECT_NAME})


  