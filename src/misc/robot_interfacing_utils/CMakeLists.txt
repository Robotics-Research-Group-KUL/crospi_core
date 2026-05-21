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
# set(BOOST_ROOT "${_BOOST_1_89_CUSTOM_ROOT}")
# set(Boost_INCLUDE_DIR "${_BOOST_1_89_CUSTOM_ROOT}/include")
# message(STATUS "CMAKE_PREFIX_PATH before: ${CMAKE_PREFIX_PATH}")
# set(CMAKE_PREFIX_PATH "${_BOOST_1_89_CUSTOM_ROOT}" ${CMAKE_PREFIX_PATH})
# message(STATUS "CMAKE_PREFIX_PATH after: ${CMAKE_PREFIX_PATH}")
# message(STATUS "Using Boost version ${Boost_VERSION}")
# find_package(Boost 1.89 REQUIRED)
# -- end -- Stuff for triple buffer implementation of boost 1.89


include_directories(include 
${LUA_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
)


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
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LUA_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS})


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

ament_export_dependencies(${dependencies} ${rosdependencies})


install(
  TARGETS robot_interfacing_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


