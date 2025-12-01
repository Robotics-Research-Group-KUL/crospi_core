cmake_minimum_required(VERSION 3.8)
project(crospi_core)

message("Compiling package ${PROJECT_SOURCE_DIR}")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options( -Wno-deprecated -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable -Wno-reorder)
endif()

# -- start --Stuff for triple buffer implementation of boost 1.89
set(BOOST_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
message(STATUS "CMAKE_PREFIX_PATH before: ${CMAKE_PREFIX_PATH}")
set(CMAKE_PREFIX_PATH "${HOME}/.local/boost/1.89.0" ${CMAKE_PREFIX_PATH})
message(STATUS "CMAKE_PREFIX_PATH after: ${CMAKE_PREFIX_PATH}")
message(STATUS "Using Boost version ${Boost_VERSION}")

find_package(Boost 1.89 REQUIRED)
# -- end -- Stuff for triple buffer implementation of boost 1.89


add_definitions(-DBOOST_DISABLE_PRAGMA_MESSAGE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(pluginlib REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   FILES hook/package_crospi_core_lua_configure_paths.sh
#   DESTINATION share/${PROJECT_NAME}/hook
# )

ament_environment_hooks("${CMAKE_SOURCE_DIR}/env-hooks/package_crospi_core_lua_configure_paths.sh.in") 

ament_export_include_directories(
  include
)


include_directories(
   include
)

# TODO: Delete the following. It is only used for debug in VSCODE:
# include_directories("/home/santiregui/etasl-cmake/standalone-cmake/install/include")

# rosidl_generate_interfaces(${PROJECT_NAME}
#   # "msg/Num.msg"
#   "msg/Test.msg"
#   "srv/TaskSpecificationString.srv"
#   "srv/TaskSpecificationFile.srv"
#   DEPENDENCIES std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
# )

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


# install(
#   TARGETS crospi_node_utils
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


# Install the Python package
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
# ament_python_install_package(scripts/python/crospi_py)
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
