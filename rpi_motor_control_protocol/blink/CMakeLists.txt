# blink/CMakeLists.txt

#########
# blink #
#########

add_executable(blink blink.c)

pico_set_program_name(blink "blink")
pico_set_program_version(blink "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(blink 0)
pico_enable_stdio_usb(blink 1)

# Add the standard library to the build
target_link_libraries(blink
  pico_stdlib
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)

# Add the standard include files to the build
target_include_directories(blink PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/
  ${CMAKE_CURRENT_LIST_DIR}/..
)

pico_add_extra_outputs(blink)


##################
# blink_priority #
##################

add_executable(blink_priority blink_priority.c)

pico_set_program_name(blink_priority "blink_priority")
pico_set_program_version(blink_priority "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(blink_priority 0)
pico_enable_stdio_usb(blink_priority 1)

# Add the standard library to the build
target_link_libraries(blink_priority
  pico_stdlib
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)

# Add the standard include files to the build
target_include_directories(blink_priority PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/
  ${CMAKE_CURRENT_LIST_DIR}/..
)

pico_add_extra_outputs(blink_priority)
