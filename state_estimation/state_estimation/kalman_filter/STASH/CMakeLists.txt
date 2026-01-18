cmake_minimum_required(VERSION 3.10)
project(kalman_filter)

# Set default C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message (STATUS "Clang compiler")
  add_compile_options(-Wall -Wextra -Wpedantic)
else()
  message (STATUS "Compiler is ${CMAKE_CXX_COMPILER_ID}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")

# Find dependencies
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (system_models REQUIRED)


########
# DATA #
########
file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR})


#############
# LIBRARIES #
#############
message("Build Kalman Filter library")

# linear_kalman_filter
add_library(linear_kalman_filter STATIC src/linear_kalman_filter.cpp)
target_include_directories(linear_kalman_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(linear_kalman_filter PUBLIC system_models::system_models Eigen3::Eigen)


#########
# TESTS #
#########

# test_linear_kalman_filter
add_executable(test_linear_kalman_filter tests/test_linear_kalman_filter.cpp)
target_link_libraries(test_linear_kalman_filter PUBLIC linear_kalman_filter gtest gtest_main)


# Enable testing with CTest
enable_testing()




################################################################################
#                                     ROS2                                     #
################################################################################

if(DEFINED ENV{ROS_VERSION})
    message("Build ROS2 targets")

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)


    ###################
    # C++ executables #
    ###################


    ###########
    # Install #
    ###########


    ###########
    # Testing #
    ###########
    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # the following line skips the linter which checks for copyrights
        # comment the line when a copyright and license is added to all source files
        set(ament_cmake_copyright_FOUND TRUE)
        # the following line skips cpplint (only works in a git repo)
        # comment the line when this package is in a git repo and when
        # a copyright and license is added to all source files
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()

endif()