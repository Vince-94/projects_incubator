cmake_minimum_required(VERSION 3.20)
project(
    a_star_algo
    VERSION 0.1.0
    DESCRIPTION "Path Planning A* Algorithms"
    LANGUAGES CXX
)


#############
# LIBRARIES #
#############

# a_star
add_library(a_star STATIC src/a_star.cpp)
target_include_directories(a_star PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# target_link_libraries(a_star PUBLIC Eigen3::Eigen)
set_target_properties(a_star PROPERTIES POSITION_INDEPENDENT_CODE ON)



###############
# EXECUTABLES #
###############

# main_a_star
add_executable(main_a_star
    main.cpp
)
target_include_directories(main_a_star PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(main_a_star PRIVATE a_star)



