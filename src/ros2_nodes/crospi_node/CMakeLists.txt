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
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(etasl REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(crospi_core REQUIRED) #Needed to read srv and msgs generated with rosidl_generate_interfaces

# -- start --Stuff for triple buffer implementation of boost 1.89
set(BOOST_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local/lib/boost/1.89.0" ${CMAKE_PREFIX_PATH})
find_package(Boost 1.89 REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")

# -- end -- Stuff for triple buffer implementation of boost 1.89

include_directories(.)

add_executable(crospi_node crospi_node.cpp)

# add_executable(crospi_node src/crospi_node.cpp)

target_link_libraries(crospi_node 
        etasl::expressiongraph_context 
        etasl::expressiongraph_solver_qpoases 
        etasl::expressiongraph_context_lua 
        etasl_misc_lib 
        crospi_utils
        crospi_node_utils
        robot_interfacing_utils
        # buffer_utils
        # triple_buffer
        # kuka_iiwa_driver
        # simple_kinematic_simulation
        )

# ament_target_dependencies(crospi_node rclcpp std_msgs sensor_msgs)

ament_target_dependencies(crospi_node
  # "crospi_core" #Needed to read srv and msgs generated with rosidl_generate_interfaces
  "rclcpp"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "pluginlib"
)


# TODO: change the directories of  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> and $<INSTALL_INTERFACE:include>
target_include_directories(crospi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})

target_compile_features(crospi_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS crospi_node
  DESTINATION lib/${PROJECT_NAME})


  