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



message("Compiling misc/simple_kinematic_simulation libraries ${CMAKE_CURRENT_SOURCE_DIR}")



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(pluginlib REQUIRED)


set(CMAKE_CXX_STANDARD 17)


set(src_simple_kinematic_simulation
    simple_kinematic_simulation.cpp
)

set(rosdependencies
  crospi_core
  pluginlib
)

set(dependencies
    fmt   
    # pugixml
    jsoncpp
    robot_interfacing_utils
)

set(ros_targets
  crospi_core::crospi_core
  pluginlib::pluginlib
)

add_library(simple_kinematic_simulation SHARED ${src_simple_kinematic_simulation})

target_include_directories(simple_kinematic_simulation 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/simple_kinematic_simulation>
  $<INSTALL_INTERFACE:include/simple_kinematic_simulation>
  ${LUA_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS})

ament_export_dependencies(crospi_core)

ament_export_libraries(
  simple_kinematic_simulation
  )
  
  install(
    TARGETS ur10_e_driver_etasl
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )


#The following is done because ROS2 deprecated ament_target_dependencies in favor of target_link_libraries, but the former is still required for older (legacy) ROS2 versions. 
if(DEFINED ENV{ROS_DISTRO} AND (
  "$ENV{ROS_DISTRO}" STREQUAL "foxy" OR
  "$ENV{ROS_DISTRO}" STREQUAL "galactic" OR
  "$ENV{ROS_DISTRO}" STREQUAL "humble" OR
  "$ENV{ROS_DISTRO}" STREQUAL "iron" OR
  "$ENV{ROS_DISTRO}" STREQUAL "jazzy"
))
      ament_target_dependencies(simple_kinematic_simulation ${rosdependencies})
      target_link_libraries(simple_kinematic_simulation ${dependencies})
      message(STATUS "Legacy ROS_DISTRO is $ENV{ROS_DISTRO}, using ament_target_dependencies")
else()
    target_link_libraries(simple_kinematic_simulation  ${dependencies} ${ros_targets})
    message(STATUS "Modern ROS_DISTRO detected ($ENV{ROS_DISTRO}), using target_link_libraries")
endif()



# install(
#   TARGETS simple_kinematic_simulation
#   EXPORT export_simple_kinematic_simulation
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


