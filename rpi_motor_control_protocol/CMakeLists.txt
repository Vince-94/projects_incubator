# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
project(freertos_pico2 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

include(FreeRTOS_Kernel_import.cmake)


#############
# LIBRARIES #
#############

add_subdirectory(pid)

add_subdirectory(blink)

add_subdirectory(pwm)

add_subdirectory(oneshot)

add_subdirectory(dshot)


#############
# MICRO_ROS #
#############

if(DEFINED ENV{ROS_VERSION})

  find_package(ament_cmake REQUIRED)
  find_package(rcl REQUIRED)
  find_package(rclc REQUIRED)
  find_package(rclc_parameter REQUIRED)
  find_package(rmw_microxrcedds REQUIRED)
  find_package(std_msgs REQUIRED)

  # main
  add_executable(main main.c)

  target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}/oneshot/include
  )

  pico_set_program_name(main "main")
  pico_set_program_version(main "0.1")

  pico_enable_stdio_uart(main 0)
  pico_enable_stdio_usb(main 1)

  target_link_libraries(main
    oneshot125
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
    rclc rcl rcutils std_msgs__rosidl_typesupport_c
  )

  pico_add_extra_outputs(main)

  ament_package()

endif()