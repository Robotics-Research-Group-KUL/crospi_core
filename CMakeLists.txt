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

cmake_minimum_required(VERSION 3.10)
project(crospi_core)

message("Compiling package ${PROJECT_SOURCE_DIR}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options( -Wno-deprecated -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable -Wno-reorder)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# Dependencies of the exported crospi_core libraries. They are found once here and exported with
# ament_export_dependencies (below), so a package that links a crospi_core target inherits all of them
# with just find_package(crospi_core). Keep this list in sync with the <depend> entries of package.xml.
set(crospi_core_exported_dependencies
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  pluginlib
  resource_retriever
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  crospi_interfaces
  etasl
  Eigen3
  jsoncpp
  fmt
)
foreach(dependency IN LISTS crospi_core_exported_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(Lua51 REQUIRED)

add_definitions(-DBOOST_DISABLE_PRAGMA_MESSAGE)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_environment_hooks("${CMAKE_SOURCE_DIR}/env-hooks/package_crospi_core_lua_configure_paths.sh.in") 

ament_export_include_directories(
  include
)

add_subdirectory(src/misc)

# Umbrella target with everything needed to write a crospi plugin (input/output handlers, robot drivers and
# simulators): target_link_libraries(my_plugin PUBLIC crospi_core::crospi_core)
add_library(crospi_core INTERFACE)
target_link_libraries(crospi_core INTERFACE crospi_utils crospi_node_utils robot_interfacing_utils)
install(TARGETS crospi_core EXPORT export_${PROJECT_NAME})

# add_subdirectory(src/ros2_nodes/simple_crospi_node)
add_subdirectory(src/ros2_nodes/crospi_node)
# add_subdirectory(src/ros2_nodes/shared_memory_test_mutex)
# add_subdirectory(src/ros2_nodes/shared_memory_test)
# add_subdirectory(src/ros2_nodes/shared_memory_test_boost)
# add_subdirectory(src/ros2_nodes/shared_memory_test_new_drivers)
# add_subdirectory(src/ros2_nodes/test_publisher_node)
# add_subdirectory(src/ros2_nodes/erwin_node)

ament_export_dependencies(ament_cmake ${crospi_core_exported_dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Install the Python package
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
ament_python_install_package(crospi_py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
