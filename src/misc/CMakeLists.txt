message("Compiling misc libraries ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(etasl REQUIRED)

add_library(etasl_misc_lib featurevariableinitializer.cpp IO_handlers.cpp )

target_link_libraries(etasl_misc_lib etasl::expressiongraph_context etasl::expressiongraph_solver_qpoases etasl::expressiongraph_context_lua )

# add_executable(etasl_node src/etasl_node.cpp)


target_include_directories(etasl_misc_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(etasl_misc_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS etasl_misc_lib
  DESTINATION lib/${PROJECT_NAME})


  