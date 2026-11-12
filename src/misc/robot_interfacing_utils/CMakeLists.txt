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

# Dependencies are found (and exported) in the top-level CMakeLists.txt.
# PUBLIC: packages linking crospi_core::robot_interfacing_utils inherit these dependencies.
add_library(robot_interfacing_utils SHARED
    thread_manager.cpp
)

target_include_directories(robot_interfacing_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LUA_INCLUDE_DIR})

target_link_libraries(robot_interfacing_utils PUBLIC
  fmt::fmt
  jsoncpp
)

install(
  TARGETS robot_interfacing_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
