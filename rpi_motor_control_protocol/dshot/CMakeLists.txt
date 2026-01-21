# dshot/CMakeLists.txt

#########
# dshot #
#########

add_library(dshot_bitbang STATIC dshot_bitbang.c)
target_include_directories(dshot_bitbang PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/
  ${CMAKE_CURRENT_LIST_DIR}/..
)




# ###############
# # dshot_16bit #
# ###############

# add_library(dshot_16bit STATIC src/dshot_16bit.c)
# target_include_directories(dshot_16bit PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
# target_link_libraries(dshot_16bit
#   pico_stdlib
#   hardware_gpio
#   hardware_pwm
#   hardware_irq
#   FreeRTOS-Kernel
#   FreeRTOS-Kernel-Heap4
# )

# pico_add_extra_outputs(dshot_16bit)

# #############
# # dshot_pio #
# #############

# add_library(dshot_pio STATIC src/dshot_pio.c)
# target_include_directories(dshot_pio PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
# target_link_libraries(dshot_pio
#   pico_stdlib
#   hardware_gpio
#   hardware_pwm
#   hardware_irq
#   FreeRTOS-Kernel
#   FreeRTOS-Kernel-Heap4
# )

# # Add PIO file (this will compile into dshot.pio.h and an object)
# pico_generate_pio_header(dshot_pio ${CMAKE_CURRENT_LIST_DIR}/src/dshot.pio)

# pico_add_extra_outputs(dshot_pio)

# #############
# # dshot_dma #
# #############

# add_library(dshot_dma STATIC src/dshot_dma.c)
# target_include_directories(dshot_dma PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
# target_link_libraries(dshot_dma
#   pico_stdlib
#   hardware_gpio
#   hardware_pwm
#   hardware_irq
#   FreeRTOS-Kernel
#   FreeRTOS-Kernel-Heap4
# )

# # Add PIO file (this will compile into dshot.pio.h and an object)
# pico_generate_pio_header(dshot_dma ${CMAKE_CURRENT_LIST_DIR}/src/dshot.pio)

# pico_add_extra_outputs(dshot_dma)


# #########
# # dshot #
# #########

# set(DSHOT_IMPL "pio" CACHE STRING "DSHOT implementation: '16bit', 'pio', 'dma'")

# add_executable(dshot dshot.c)
# target_include_directories(dshot PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
# target_link_libraries(dshot PRIVATE pico_stdlib)

# # Link only the selected dshot implementation
# if(DSHOT_IMPL STREQUAL "16bit")
#     target_compile_definitions(dshot PRIVATE DSHOT_USE_16BIT=0)
#     target_link_libraries(dshot PRIVATE dshot_16bit)
# elseif(DSHOT_IMPL STREQUAL "pio")
#     target_compile_definitions(dshot PRIVATE DSHOT_USE_PIO=1)
#     target_link_libraries(dshot PRIVATE dshot_pio)
# elseif(DSHOT_IMPL STREQUAL "dma")
#     target_compile_definitions(dshot PRIVATE DSHOT_USE_PIO=2)
#     target_link_libraries(dshot PRIVATE dshot_dma)
# else()
#     message(FATAL_ERROR "Unknown DSHOT_IMPL: ${DSHOT_IMPL} (use '16bit', 'pio', 'dma')")
# endif()

# pico_add_extra_outputs(dshot)
