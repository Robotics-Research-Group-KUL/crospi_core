cmake_minimum_required(VERSION 3.8)
project(etasl_ros2)

message("Compiling package ${PROJECT_SOURCE_DIR}")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options( -Wno-deprecated -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable -Wno-reorder)
endif()

add_definitions(-DBOOST_DISABLE_PRAGMA_MESSAGE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(DIRECTORY launch robot_description etasl scripts
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)


include_directories(
   include
)

# include_directories(
#    src/misc/kuka_iiwa_driver
# ) #TODO: Remove this

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
add_subdirectory(src/ros2_nodes/simple_etasl_node)
add_subdirectory(src/ros2_nodes/etasl_node)
add_subdirectory(src/ros2_nodes/test_publisher_node)
add_subdirectory(src/ros2_nodes/erwin_node)



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
