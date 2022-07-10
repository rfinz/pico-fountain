add_executable(fountain)

pico_generate_pio_header(fountain ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(fountain PRIVATE fountain.c)

target_compile_definitions(fountain PRIVATE
        PIN_DBG1=3)

target_link_libraries(fountain PRIVATE pico_stdlib hardware_pio hardware_dma hardware_i2c hardware_adc)
pico_add_extra_outputs(fountain)
