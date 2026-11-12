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

message("Compiling misc/crospi_utils libraries ${CMAKE_CURRENT_SOURCE_DIR}")

# Dependencies are found (and exported) in the top-level CMakeLists.txt.
# PUBLIC: packages linking crospi_core::crospi_utils inherit these dependencies.
set(crospi_utils_dependencies
  fmt::fmt
  jsoncpp
  Eigen3::Eigen
  etasl::expressiongraph_context
  etasl::expressiongraph_solver_qpoases
  etasl::expressiongraph_context_lua
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_action::rclcpp_action
  resource_retriever::resource_retriever
  ament_index_cpp::ament_index_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  pluginlib::pluginlib
  ${crospi_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

add_library(crospi_utils SHARED
    featurevariableinitializer.cpp
    outputhandler.cpp #TODO: Remove after pasing cut_global method somewhere else
    blackboard.cpp
    string_interpolate.cpp
    qpoasessolverfactory.cpp
    json_checker.cpp
)

target_include_directories(crospi_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LUA_INCLUDE_DIR})

target_link_libraries(crospi_utils PUBLIC ${crospi_utils_dependencies})

install(
  TARGETS crospi_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------- FOR AMENT LUA PACKAGE ---------------------
add_library(amentlua SHARED amentlua.cpp)
target_link_libraries(amentlua PUBLIC ${crospi_utils_dependencies})

install(TARGETS amentlua
    DESTINATION lib)
