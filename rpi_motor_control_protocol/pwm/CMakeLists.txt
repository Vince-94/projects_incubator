# blink/CMakeLists.txt

#######
# pwm #
#######

add_executable(pwm pwm.c)

pico_set_program_name(pwm "pwm")
pico_set_program_version(pwm "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pwm 0)
pico_enable_stdio_usb(pwm 1)

# Add the standard library to the build
target_link_libraries(pwm
  pid
  pico_stdlib
  hardware_gpio
  hardware_pwm
  hardware_irq
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)

# Add the standard include files to the build
target_include_directories(pwm PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/
  ${CMAKE_CURRENT_LIST_DIR}/..
)

pico_add_extra_outputs(pwm)


###########
# pwm_open #
###########

add_executable(pwm_open pwm_open.c)

pico_set_program_name(pwm_open "pwm_open")
pico_set_program_version(pwm_open "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pwm_open 0)
pico_enable_stdio_usb(pwm_open 1)

# Add the standard library to the build
target_link_libraries(pwm_open
  pico_stdlib
  hardware_gpio
  hardware_pwm
  hardware_irq
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)

# Add the standard include files to the build
target_include_directories(pwm_open PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/
  ${CMAKE_CURRENT_LIST_DIR}/..
)

pico_add_extra_outputs(pwm_open)
