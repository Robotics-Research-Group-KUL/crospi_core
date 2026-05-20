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

message("Compiling misc/robot_interfacing_utils libraries ${CMAKE_CURRENT_SOURCE_DIR}")



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(etasl REQUIRED)
# find_package(resource_retriever REQUIRED)
# find_package(crospi_interfaces REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(Lua51 REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(pugixml REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(Sphinx REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
#add_subdirectory( doc )


set(CMAKE_CXX_STANDARD 17)

# -- start --Stuff for triple buffer implementation of boost 1.89
set(BOOST_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local/lib/boost/1.89.0" ${CMAKE_PREFIX_PATH})
find_package(Boost 1.89 REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
# -- end -- Stuff for triple buffer implementation of boost 1.89


include_directories(include ${LUA_INCLUDE_DIR})


set(src_robot_interfacing_utils
    thread_manager.cpp
    # simulationrobotdriver.cpp
    # simulationrobotdriverfactory.cpp
)

set(rosdependencies)
set(ros_targets)

set(dependencies
    fmt   
    # pugixml
    jsoncpp
)


add_library(robot_interfacing_utils SHARED ${src_robot_interfacing_utils})

target_include_directories(robot_interfacing_utils 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/robot_interfacing_utils>
  $<INSTALL_INTERFACE:include/robot_interfacing_utils>)


ament_export_libraries(
  robot_interfacing_utils
)

#The following is done because ROS2 deprecated ament_target_dependencies in favor of target_link_libraries, but the former is still required for older (legacy) ROS2 versions. 

if(DEFINED ENV{ROS_DISTRO} AND (
  "$ENV{ROS_DISTRO}" STREQUAL "foxy" OR
  "$ENV{ROS_DISTRO}" STREQUAL "galactic" OR
  "$ENV{ROS_DISTRO}" STREQUAL "humble" OR
  "$ENV{ROS_DISTRO}" STREQUAL "iron" OR
  "$ENV{ROS_DISTRO}" STREQUAL "jazzy"
))
      ament_target_dependencies(robot_interfacing_utils ${rosdependencies})
      target_link_libraries(robot_interfacing_utils ${dependencies})
      message(STATUS "Legacy ROS_DISTRO is $ENV{ROS_DISTRO}, using ament_target_dependencies")
else()
    target_link_libraries(robot_interfacing_utils  ${dependencies} ${ros_targets})
    message(STATUS "Modern ROS_DISTRO detected ($ENV{ROS_DISTRO}), using target_link_libraries")
endif()


install(
  TARGETS robot_interfacing_utils
  EXPORT export_robot_interfacing_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


