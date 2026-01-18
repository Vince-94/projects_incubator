cmake_minimum_required(VERSION 3.20)
project(
    kalman_filter
    VERSION 0.1.0
    DESCRIPTION "Kalman Filter estimator"
    LANGUAGES CXX
)


#############
# LIBRARIES #
#############

# linear_kalman_filter
add_library(linear_kalman_filter STATIC src/linear_kalman_filter.cpp)
target_include_directories(linear_kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(linear_kalman_filter PUBLIC Eigen3::Eigen)
set_target_properties(linear_kalman_filter PROPERTIES POSITION_INDEPENDENT_CODE ON)


# extended_kalman_filter
add_library(extended_kalman_filter STATIC src/extended_kalman_filter.cpp)
target_include_directories(extended_kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(extended_kalman_filter PUBLIC Eigen3::Eigen)
set_target_properties(extended_kalman_filter PROPERTIES POSITION_INDEPENDENT_CODE ON)


# unscented_kalman_filter
add_library(unscented_kalman_filter STATIC src/unscented_kalman_filter.cpp)
target_include_directories(unscented_kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(unscented_kalman_filter PUBLIC Eigen3::Eigen)
set_target_properties(unscented_kalman_filter PROPERTIES POSITION_INDEPENDENT_CODE ON)




###############
# EXECUTABLES #
###############

# main
add_executable(main
    main.cpp
)
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(main PRIVATE linear_kalman_filter)



