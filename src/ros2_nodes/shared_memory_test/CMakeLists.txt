message("Compiling node located in ${CMAKE_CURRENT_SOURCE_DIR}")


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(etasl REQUIRED)

message(STATUS "===================CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")


list(APPEND CMAKE_MODULE_PATH "/usr/local/share/cmake/low_latency_buffers")
find_package(low_latency_buffers REQUIRED) #enables buffer_utils and triple_buffer

set(BOOST_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local/lib/boost/1.89.0" ${CMAKE_PREFIX_PATH})

find_package(Boost 1.89 REQUIRED)

message(STATUS "Using Boost version ${Boost_VERSION}")



add_executable(shared_memory_test shared_memory_test.cpp shared_memory_test.hpp)
# add_executable(shared_memory_test src/shared_memory_test.cpp)

# message(STATUS "===================buffer_utils_libraries: ${buffer_utils_LIBRARIES}")
# message(STATUS "=====================triple_buffer_LIBRARIES: ${triple_buffer_LIBRARIES}")

target_link_libraries(shared_memory_test 
        etasl_misc_lib 
        crospi_utils
        crospi_node_utils
        robot_interfacing_utils
        # kuka_iiwa_driver
        # simple_kinematic_simulation
        buffer_utils
        triple_buffer
        # ${buffer_utils_LIBRARIES}
        # ${triple_buffer_LIBRARIES}
        )


ament_target_dependencies(shared_memory_test rclcpp std_msgs sensor_msgs)

# target_include_directories(shared_memory_test
#     PRIVATE
#         ${buffer_common_INCLUDE_DIRS}
#         ${buffer_utils_INCLUDE_DIRS}
#         ${triple_buffer_INCLUDE_DIRS}
# )


target_include_directories(shared_memory_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(shared_memory_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS shared_memory_test
  DESTINATION lib/${PROJECT_NAME})


  