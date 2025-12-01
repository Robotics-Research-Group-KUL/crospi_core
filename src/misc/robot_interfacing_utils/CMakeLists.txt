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
set(BOOST_ROOT "$ENV{HOME}/.local/lib/boost/1.89.0")
set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local/lib/boost/1.89.0" ${CMAKE_PREFIX_PATH})
find_package(Boost 1.89 REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
# -- end -- Stuff for triple buffer implementation of boost 1.89


include_directories(include ${LUA_INCLUDE_DIR})


set(src_robot_interfacing_utils
    thread_manager.cpp
    # simulationrobotdriver.cpp
    # simulationrobotdriverfactory.cpp
)

set(rosdependencies)

set(dependencies
    fmt   
    # pugixml
    jsoncpp
)


add_library(robot_interfacing_utils SHARED ${src_robot_interfacing_utils})

target_include_directories(robot_interfacing_utils 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/robot_interfacing_utils>
  $<INSTALL_INTERFACE:include/robot_interfacing_utils>)


ament_export_libraries(
  robot_interfacing_utils
)



ament_target_dependencies(robot_interfacing_utils ${rosdependencies})  

target_link_libraries(robot_interfacing_utils ${dependencies})

install(
  TARGETS robot_interfacing_utils
  EXPORT export_robot_interfacing_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


