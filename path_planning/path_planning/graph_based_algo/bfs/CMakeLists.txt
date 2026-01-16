cmake_minimum_required(VERSION 3.20)
project(
    bfs_algo
    VERSION 0.1.0
    DESCRIPTION "Path Planning Breadth-First Search Algorithms"
    LANGUAGES CXX
)


#############
# LIBRARIES #
#############

# bfs
add_library(bfs STATIC src/bfs.cpp)
target_include_directories(bfs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# target_link_libraries(bfs PUBLIC Eigen3::Eigen)
set_target_properties(bfs PROPERTIES POSITION_INDEPENDENT_CODE ON)



###############
# EXECUTABLES #
###############

# main_bfs
add_executable(main_bfs
    main.cpp
)
target_include_directories(main_bfs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(main_bfs PRIVATE bfs)



