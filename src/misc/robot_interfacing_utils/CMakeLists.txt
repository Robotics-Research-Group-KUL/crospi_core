message("Compiling misc/etasl_interfacing_utils libraries ${CMAKE_CURRENT_SOURCE_DIR}")



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

set(CMAKE_CXX_STANDARD 17)

include_directories(include ${LUA_INCLUDE_DIR})


set(src_etasl_interfacing_utils
    thread_manager.cpp
    simulationrobotdriver.cpp
    simulationrobotdriverfactory.cpp
)

set(rosdependencies)

set(dependencies
    fmt   
    # pugixml
    jsoncpp
)


add_library(etasl_interfacing_utils ${src_etasl_interfacing_utils})

target_include_directories(etasl_interfacing_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(etasl_interfacing_utils ${rosdependencies})  
target_link_libraries(etasl_interfacing_utils ${dependencies})

install(TARGETS etasl_interfacing_utils
  DESTINATION lib/${PROJECT_NAME})
