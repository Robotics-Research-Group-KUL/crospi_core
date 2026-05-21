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

# -- start --Stuff for triple buffer implementation of boost 1.89
set(Boost_NO_SYSTEM_PATHS ON)
# --------------------------------------------------
# Resolve Boost 1.89 root. This is done because in github, docker, and local home might or might not be defined.
# If BOOST_189_ROOT is defined in the environment, use that installation directory instead (e.g. as done manually for github actions).
# --------------------------------------------------
if(DEFINED ENV{BOOST_189_ROOT} AND NOT "$ENV{BOOST_189_ROOT}" STREQUAL "")
  set(_BOOST_1_89_CUSTOM_ROOT "$ENV{BOOST_189_ROOT}")
  message(STATUS "Using Boost 1.89 from BOOST_189_ROOT=${_BOOST_1_89_CUSTOM_ROOT}")
else()
  if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
    set(_BOOST_1_89_CUSTOM_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
    message(STATUS "Using Boost 1.89 from HOME fallback: ${_BOOST_1_89_CUSTOM_ROOT}")
  else()
    message(FATAL_ERROR
      "Boost 1.89 requested, but neither BOOST_189_ROOT nor HOME is defined")
  endif()
endif()

set(BOOST_ROOT "${_BOOST_1_89_CUSTOM_ROOT}")
set(Boost_INCLUDE_DIR "${_BOOST_1_89_CUSTOM_ROOT}/include")
message(STATUS "CMAKE_PREFIX_PATH before: ${CMAKE_PREFIX_PATH}")
set(CMAKE_PREFIX_PATH "${_BOOST_1_89_CUSTOM_ROOT}" ${CMAKE_PREFIX_PATH})
message(STATUS "CMAKE_PREFIX_PATH after: ${CMAKE_PREFIX_PATH}")
message(STATUS "Using Boost version ${Boost_VERSION}")

find_package(Boost 1.89 REQUIRED)
ament_export_dependencies(Boost)

message(STATUS "Boost include dirs after find_package: ${Boost_INCLUDE_DIRS}")
# -- end -- Stuff for triple buffer implementation of boost 1.89


add_executable(crospi_node crospi_node.cpp)



set(rosdependencies 
    rclcpp
    rclcpp_lifecycle
    pluginlib
    std_msgs
    std_srvs
    sensor_msgs
    lifecycle_msgs
)

set(dependencies
        etasl::expressiongraph_context 
        etasl::expressiongraph_solver_qpoases 
        etasl::expressiongraph_context_lua 
        etasl_misc_lib 
        crospi_utils
        crospi_node_utils
        robot_interfacing_utils
)

set(ros_targets 
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    pluginlib::pluginlib
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
)


#The following is done because ROS2 deprecated ament_target_dependencies in favor of target_link_libraries, but the former is still required for older (legacy) ROS2 versions. 
if(DEFINED ENV{ROS_DISTRO} AND (
  "$ENV{ROS_DISTRO}" STREQUAL "foxy" OR
  "$ENV{ROS_DISTRO}" STREQUAL "galactic" OR
  "$ENV{ROS_DISTRO}" STREQUAL "humble" OR
  "$ENV{ROS_DISTRO}" STREQUAL "iron" OR
  "$ENV{ROS_DISTRO}" STREQUAL "jazzy"
))
      ament_target_dependencies(crospi_node ${rosdependencies})
      target_link_libraries(crospi_node ${dependencies})
      message(STATUS "Legacy ROS_DISTRO is $ENV{ROS_DISTRO}, using ament_target_dependencies")
else()
    target_link_libraries(crospi_node  ${dependencies} ${ros_targets})
    message(STATUS "Modern ROS_DISTRO detected ($ENV{ROS_DISTRO}), using target_link_libraries")
endif()


# TODO: change the directories of  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> and $<INSTALL_INTERFACE:include>
target_include_directories(crospi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LUA_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS})

target_compile_features(crospi_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS crospi_node
  DESTINATION lib/${PROJECT_NAME})


  