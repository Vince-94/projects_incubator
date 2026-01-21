# oneshot/CMakeLists.txt

##############
# oneshot125 #
##############

add_library(oneshot125 STATIC src/oneshot125.c)

target_include_directories(oneshot125 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Link to pico / rtos / hardware libs as needed:
target_link_libraries(oneshot125
  pico_stdlib
  hardware_gpio
  hardware_pwm
  hardware_irq
  hardware_adc
#   FreeRTOS-Kernel
#   FreeRTOS-Kernel-Heap4
)

# Install targets / headers
install(TARGETS oneshot125 DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
