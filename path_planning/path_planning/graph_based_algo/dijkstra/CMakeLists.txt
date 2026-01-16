cmake_minimum_required(VERSION 3.20)
project(
    dijkstra_algo
    VERSION 0.1.0
    DESCRIPTION "Path Planning Dijkstra Algorithms"
    LANGUAGES CXX
)


#############
# LIBRARIES #
#############

# dijkstra
add_library(dijkstra STATIC src/dijkstra.cpp)
target_include_directories(dijkstra PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# target_link_libraries(dijkstra PUBLIC Eigen3::Eigen)
set_target_properties(dijkstra PROPERTIES POSITION_INDEPENDENT_CODE ON)



###############
# EXECUTABLES #
###############

# main_dijkstra
add_executable(main_dijkstra
    main.cpp
)
target_include_directories(main_dijkstra PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(main_dijkstra PRIVATE dijkstra)



