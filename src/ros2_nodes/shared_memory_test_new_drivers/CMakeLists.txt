#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.


message("Compiling node located in ${CMAKE_CURRENT_SOURCE_DIR}")


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(etasl REQUIRED)

message(STATUS "===================CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")


# list(APPEND CMAKE_MODULE_PATH "/usr/local/share/cmake/low_latency_buffers")
# find_package(low_latency_buffers REQUIRED) #enables buffer_utils and triple_buffer

set(BOOST_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local/lib/boost/1.89.0" ${CMAKE_PREFIX_PATH})

find_package(Boost 1.89 REQUIRED)

message(STATUS "Using Boost version ${Boost_VERSION}")



add_executable(shared_memory_test_new_drivers shared_memory_test_new_drivers.cpp shared_memory_test_new_drivers.hpp)
# add_executable(shared_memory_test_new_drivers src/shared_memory_test_new_drivers.cpp)

# message(STATUS "===================buffer_utils_libraries: ${buffer_utils_LIBRARIES}")
# message(STATUS "=====================triple_buffer_LIBRARIES: ${triple_buffer_LIBRARIES}")

target_link_libraries(shared_memory_test_new_drivers 
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


ament_target_dependencies(shared_memory_test_new_drivers rclcpp std_msgs sensor_msgs)

# target_include_directories(shared_memory_test_new_drivers
#     PRIVATE
#         ${buffer_common_INCLUDE_DIRS}
#         ${buffer_utils_INCLUDE_DIRS}
#         ${triple_buffer_INCLUDE_DIRS}
# )


target_include_directories(shared_memory_test_new_drivers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(shared_memory_test_new_drivers PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS shared_memory_test_new_drivers
  DESTINATION lib/${PROJECT_NAME})


  