cmake_minimum_required(VERSION 3.20)

project(
    path_planning
    VERSION 0.1.0
    DESCRIPTION "Path Planning stack"
    LANGUAGES CXX
)


# ------------------------------------------------------------------------------
# Global options
# ------------------------------------------------------------------------------
# Set default C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

# Define compiler options
set(GCC_CLANG_FLAGS "-Wall" "-Wextra" "-Wpedantic" "-Werror")

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(STATUS "${CMAKE_CXX_COMPILER_ID} compiler detected")
  add_compile_options(${GCC_CLANG_FLAGS})
else()
  message(STATUS "Compiler is ${CMAKE_CXX_COMPILER_ID}")
endif()

# Set default build type
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")


# ------------------------------------------------------------------------------
# Build type (default to Release)
# ------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


# ------------------------------------------------------------------------------
# Warnings (per-target, not global)
# ------------------------------------------------------------------------------
set(PROJECT_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
)


# ------------------------------------------------------------------------------
# Dependencies (examples)
# ------------------------------------------------------------------------------
find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(SDL2_ttf REQUIRED)
find_path(SDL2_GFX_INCLUDE_DIR
    SDL2_gfxPrimitives.h
    PATH_SUFFIXES SDL2
)
find_library(SDL2_GFX_LIBRARY
    NAMES SDL2_gfx
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)


#############
# LIBRARIES #
#############

# visualization
# add_library(visualization
#     src/visualization.cpp
# )
# target_include_directories(visualization PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(visualization PRIVATE SDL2::SDL2 SDL2_image::SDL2_image SDL2_ttf::SDL2_ttf ${SDL2_GFX_LIBRARY})



###########
# MODULES #
###########

include_directories(${PROJECT_SOURCE_DIR}/utils/include)

add_subdirectory(graph_based_algo/bfs)
add_subdirectory(graph_based_algo/dijkstra)




# ------------------------------------------------------------------------------
# Installation
# ------------------------------------------------------------------------------
# include(GNUInstallDirs)

# install(
#     TARGETS ${PROJECT_NAME}
#     EXPORT ${PROJECT_NAME}Targets
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )

# install(
#     DIRECTORY include/
#     DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
# )


# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
# option(BUILD_TESTS "Build tests" ON)

# if(BUILD_TESTS)
#     enable_testing()
#     add_subdirectory(tests)
# endif()


