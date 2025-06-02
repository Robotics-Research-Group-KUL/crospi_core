message("Compiling misc/etasl_node_utils libraries ${CMAKE_CURRENT_SOURCE_DIR}")

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
find_package(std_msgs REQUIRED)
find_package(Lua51 REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(pugixml REQUIRED)
find_package(jsoncpp REQUIRED)
# find_package(Sphinx REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
#add_subdirectory( doc )

set(CMAKE_CXX_STANDARD 17)

include_directories(include ${LUA_INCLUDE_DIR})


set( src_etasl_node_utils
    rosloghandler.cpp
    # rostask.cpp

    topicoutputhandler.cpp
    jointstateoutputhandler.cpp
    jointstateinputhandler.cpp
    topicinputhandler.cpp
    tfoutputhandler.cpp
    twistinputhandler.cpp
    wrenchinputhandler.cpp
    tfinputhandler.cpp

    topicoutputhandlerfactory.cpp
    jointstateoutputhandlerfactory.cpp
    jointstateinputhandlerfactory.cpp    
    topicinputhandlerfactory.cpp
    tfoutputhandlerfactory.cpp
    twistinputhandlerfactory.cpp
    wrenchinputhandlerfactory.cpp
    tfinputhandlerfactory.cpp

    #robotsimulator.cpp
    robotsimulatorfactory.cpp    
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
)

set(dependencies
    fmt
    etasl::expressiongraph_context 
    etasl::expressiongraph_solver_qpoases 
    etasl::expressiongraph_context_lua    
    # pugixml
    jsoncpp
    etasl_task_utils
)


add_library(etasl_node_utils SHARED ${src_etasl_node_utils})

target_include_directories(etasl_node_utils 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/etasl_node_utils>
  $<INSTALL_INTERFACE:include/etasl_node_utils>)

ament_export_libraries(
  etasl_node_utils
)

ament_target_dependencies(etasl_node_utils ${rosdependencies})
 
target_link_libraries(etasl_node_utils ${dependencies})

install(
  TARGETS etasl_node_utils
  EXPORT export_etasl_node_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



