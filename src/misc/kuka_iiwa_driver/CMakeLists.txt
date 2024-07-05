message("Compiling misc/kuka_iiwa_driver libraries ${CMAKE_CURRENT_SOURCE_DIR}")



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

## KUKA SDK specific
set(KUKA_FRI_SDK ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/)
set(FRISDK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/include)
set(FRISDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/external/kuka_fri_sdk/lib)
add_custom_target(kuka_fri_sdk COMMAND make clean && make
                   WORKING_DIRECTORY ${KUKA_FRI_SDK}/build/GNUMake
)
include_directories(${FRISDK_INCLUDE})

# link_directories(${FRISDK_LIB})


set(src_kuka_iiwa_driver
kukaiiwa_robotdriver.cpp
kukaiiwa_robotdriverfactory.cpp
kukaiiwa_client.cpp
)


add_library(kuka_iiwa_driver ${src_kuka_iiwa_driver})

add_dependencies(kuka_iiwa_driver kuka_fri_sdk) #Makes sure to compile first kuka_fri_sdk before the kuka_iiwa_driver

find_library(FRI_CLIENT_LIB
             NAMES libFRIClient.so
             PATHS ${FRISDK_LIB}
)



set(rosdependencies)

set(dependencies
    fmt   
    # pugixml
    jsoncpp
    etasl_misc_lib 
    etasl_task_utils
    etasl_node_utils
    etasl_interfacing_utils
    # ${FRISDK_LIB}/libFRIClient.so
    ${FRI_CLIENT_LIB}
    )


    




message("--------------FRI_CLIENT_LIB: ${FRI_CLIENT_LIB}----------------")

# target_link_libraries(kuka_iiwa_driver ${FRI_CLIENT_LIB})

target_include_directories(kuka_iiwa_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(kuka_iiwa_driver ${rosdependencies})  
target_link_libraries(kuka_iiwa_driver ${dependencies})

install(TARGETS kuka_iiwa_driver
  DESTINATION lib/${PROJECT_NAME})
