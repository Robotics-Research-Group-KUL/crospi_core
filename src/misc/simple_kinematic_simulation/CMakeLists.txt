message("Compiling misc/simple_kinematic_simulation libraries ${CMAKE_CURRENT_SOURCE_DIR}")



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(etasl REQUIRED)
# find_package(resource_retriever REQUIRED)
# find_package(etasl_interfaces REQUIRED)
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
find_package(pluginlib REQUIRED)


set(CMAKE_CXX_STANDARD 17)

include_directories(include ${LUA_INCLUDE_DIR})


set(src_simple_kinematic_simulation
    simple_kinematic_simulation.cpp
)

set(rosdependencies
  "etasl_ros2"
  "pluginlib"
)


set(dependencies
    fmt   
    # pugixml
    jsoncpp
    robot_interfacing_utils
)


add_library(simple_kinematic_simulation SHARED ${src_simple_kinematic_simulation})

target_include_directories(simple_kinematic_simulation 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/simple_kinematic_simulation>
  $<INSTALL_INTERFACE:include/simple_kinematic_simulation>)

ament_export_dependencies(etasl_ros2)


ament_export_libraries(
  simple_kinematic_simulation
  )
  
  install(
    TARGETS ur10_e_driver_etasl
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )


ament_target_dependencies(simple_kinematic_simulation ${rosdependencies})  

target_link_libraries(simple_kinematic_simulation ${dependencies})



# install(
#   TARGETS simple_kinematic_simulation
#   EXPORT export_simple_kinematic_simulation
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )


