cmake_minimum_required(VERSION 3.10)
project(model_predictive_control LANGUAGES CXX)

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

# Find dependencies
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(proxsuite REQUIRED)
# find_package(yaml-cpp REQUIRED)

find_package(GTest REQUIRED)
# find_package(GMock REQUIRED)


#############
# LIBRARIES #
#############

include_directories(${PROJECT_SOURCE_DIR}/utils/include)

add_subdirectory(linear_mpc)


#########
# TESTS #
#########
enable_testing()

add_executable(test_data_structure utils/tests/test_data_structure.cpp)
target_include_directories(test_data_structure PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_data_structure PRIVATE
  linear_mpc
  GTest::gmock_main
  GTest::gtest
)

include(GoogleTest)
gtest_discover_tests(test_data_structure)
