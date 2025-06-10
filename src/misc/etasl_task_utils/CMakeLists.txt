message("Compiling misc/etasl_task_utils libraries ${CMAKE_CURRENT_SOURCE_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(etasl REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(etasl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Lua51 REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(pugixml REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(Sphinx REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

#add_subdirectory( doc )

set(CMAKE_CXX_STANDARD 17)

include_directories(include ${LUA_INCLUDE_DIR})


set(src_etasl_task_utils
    featurevariableinitializer.cpp
    outputhandler.cpp #TODO: Remove after pasing cut_global method somewhere else
    # task.cpp
    blackboard.cpp
    printobserver.cpp
    string_interpolate.cpp
    #solverregistry.cpp
    #solverfactory_qpoases.cpp
    qpoasessolverfactory.cpp
    # fileoutputhandler.cpp
    # fileoutputhandlerfactory.cpp
    json_checker.cpp
)

set(rosdependencies 
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    etasl_interfaces
    resource_retriever
    ament_index_cpp
    Eigen3
    sensor_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    pluginlib
)

set(dependencies
    fmt
    etasl::expressiongraph_context 
    etasl::expressiongraph_solver_qpoases 
    etasl::expressiongraph_context_lua    
    # pugixml
    jsoncpp
)


add_library(etasl_task_utils SHARED ${src_etasl_task_utils})

target_include_directories(etasl_task_utils 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/etasl_task_utils>
  $<INSTALL_INTERFACE:include/etasl_task_utils>)


ament_export_libraries(
  etasl_task_utils
)

ament_target_dependencies(etasl_task_utils ${rosdependencies}) 

target_link_libraries(etasl_task_utils ${dependencies})


install(
  TARGETS etasl_task_utils
  EXPORT export_etasl_task_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(amentlua SHARED amentlua.cpp )
ament_target_dependencies(amentlua ${rosdependencies})
target_link_libraries(amentlua ${dependencies})
install(TARGETS amentlua
    DESTINATION lib)
  
