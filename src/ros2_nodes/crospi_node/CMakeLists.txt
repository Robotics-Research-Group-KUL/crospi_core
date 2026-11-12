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

# The other dependencies are found in the top-level CMakeLists.txt (only needed by the node itself)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(crospi_node crospi_node.cpp)

target_link_libraries(crospi_node
  etasl_misc_lib
  crospi_utils
  crospi_node_utils
  robot_interfacing_utils
  ${std_srvs_TARGETS}
  ${lifecycle_msgs_TARGETS}
)

# TODO: change the directories of  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> and $<INSTALL_INTERFACE:include>
target_include_directories(crospi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LUA_INCLUDE_DIR})

target_compile_features(crospi_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS crospi_node
  DESTINATION lib/${PROJECT_NAME})
