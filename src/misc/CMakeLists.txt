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

message("Compiling misc libraries ${CMAKE_CURRENT_SOURCE_DIR}")

# Dependencies are found in the top-level CMakeLists.txt

add_library(etasl_misc_lib
            featurevariableinitializer.cpp
            port_observer.cpp)

target_include_directories(etasl_misc_lib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LUA_INCLUDE_DIR})

target_link_libraries(etasl_misc_lib PUBLIC
  etasl::expressiongraph_context
  etasl::expressiongraph_solver_qpoases
  etasl::expressiongraph_context_lua
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

target_compile_features(etasl_misc_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS etasl_misc_lib
  DESTINATION lib/${PROJECT_NAME})

add_subdirectory(crospi_node_utils)
add_subdirectory(crospi_utils)
add_subdirectory(robot_interfacing_utils)
