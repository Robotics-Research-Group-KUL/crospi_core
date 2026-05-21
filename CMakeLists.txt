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

cmake_minimum_required(VERSION 3.8)
project(crospi_core)

message("Compiling package ${PROJECT_SOURCE_DIR}")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options( -Wno-deprecated -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable -Wno-reorder)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

add_definitions(-DBOOST_DISABLE_PRAGMA_MESSAGE)


install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  # DESTINATION include
  DESTINATION include
)


ament_environment_hooks("${CMAKE_SOURCE_DIR}/env-hooks/package_crospi_core_lua_configure_paths.sh.in") 

ament_export_include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)


add_subdirectory(src/misc)


# add_subdirectory(src/ros2_nodes/simple_crospi_node)
add_subdirectory(src/ros2_nodes/crospi_node)
# add_subdirectory(src/ros2_nodes/shared_memory_test_mutex)
# add_subdirectory(src/ros2_nodes/shared_memory_test)
# add_subdirectory(src/ros2_nodes/shared_memory_test_boost)
# add_subdirectory(src/ros2_nodes/shared_memory_test_new_drivers)
# add_subdirectory(src/ros2_nodes/test_publisher_node)
# add_subdirectory(src/ros2_nodes/erwin_node)

ament_export_dependencies(ament_cmake)
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
