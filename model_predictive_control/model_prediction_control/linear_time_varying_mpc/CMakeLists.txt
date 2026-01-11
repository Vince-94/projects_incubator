# linear_time_varying_mpc
cmake_minimum_required(VERSION 3.10)
project(linear_time_varying_mpc LANGUAGES CXX)


#############
# LIBRARIES #
#############

# linear_time_varying_mpc
add_library(linear_time_varying_mpc STATIC src/linear_time_varying_mpc.cpp)
target_include_directories(linear_time_varying_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(linear_time_varying_mpc PUBLIC Eigen3::Eigen)
set_target_properties(linear_time_varying_mpc PROPERTIES POSITION_INDEPENDENT_CODE ON)


###########
# INSTALL #
###########

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
    linear_time_varying_mpc
  EXPORT controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


##############
# EXECUTABLE #
##############

# main
# add_executable(main main.cpp)
# target_include_directories(main PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(main ${typesupport_target})
# target_compile_features(main PUBLIC c_std_99 cxx_std_20)


#########
# TESTS #
#########
enable_testing()

# add_executable(test_linear_mpc tests/test_linear_mpc.cpp)
# target_include_directories(test_linear_mpc PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
# )
# target_link_libraries(test_linear_mpc PRIVATE
#   linear_time_varying_mpc
#   GTest::gmock_main
#   GTest::gtest
# )

# include(GoogleTest)
# gtest_discover_tests(test_linear_mpc)