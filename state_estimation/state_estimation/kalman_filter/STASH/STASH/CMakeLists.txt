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


########
# DATA #
########

file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR})


#############
# LIBRARIES #
#############
message("Build Kalman Filter library")

# utils
#add_library(utils STATIC src/utils.cpp)
#target_include_directories(utils PUBLIC include/)
#target_link_libraries(utils PUBLIC SDL2 SDL2_ttf Eigen3::Eigen)

# system_models
add_library(system_models STATIC src/system_models.cpp)
target_include_directories(system_models PUBLIC include/)
target_link_libraries(system_models PUBLIC Eigen3::Eigen)

# sensor_models
add_library(sensor_models STATIC src/sensor_models.cpp)
target_include_directories(sensor_models PUBLIC include/)
target_link_libraries(sensor_models PUBLIC Eigen3::Eigen)

# linear_kalman_filter
add_library(linear_kalman_filter STATIC src/linear_kalman_filter.cpp)
target_include_directories(linear_kalman_filter PUBLIC include/)
target_link_libraries(linear_kalman_filter PUBLIC Eigen3::Eigen)

# extended_kalman_filter
add_library(extended_kalman_filter STATIC src/extended_kalman_filter.cpp)
target_include_directories(extended_kalman_filter PUBLIC include/)
target_link_libraries(extended_kalman_filter PUBLIC sensor_models Eigen3::Eigen)


#########
# TESTS #
#########

# test_system_models
add_executable(test_system_models tests/test_system_models.cpp)
target_link_libraries(test_system_models PUBLIC system_models Eigen3::Eigen gtest gtest_main)

# test_linear_kalman_filter
add_executable(test_linear_kalman_filter tests/test_linear_kalman_filter.cpp)
target_link_libraries(test_linear_kalman_filter PUBLIC linear_kalman_filter system_models gtest gtest_main)

# test_extended_kalman_filter
add_executable(test_extended_kalman_filter tests/test_extended_kalman_filter.cpp)
target_link_libraries(test_extended_kalman_filter PUBLIC extended_kalman_filter system_models gtest gtest_main)


# Enable testing with CTest
enable_testing()

# Add tests to CTest
add_test(NAME test_system_models COMMAND ./test_system_models)
add_test(NAME test_linear_kalman_filter COMMAND ./test_linear_kalman_filter)



###############
# EXECUTABLES #
###############

# main
add_executable(main src/main.cpp)
target_link_libraries(main PUBLIC linear_kalman_filter system_models)


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