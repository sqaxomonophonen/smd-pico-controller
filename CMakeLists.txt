cmake_minimum_required(VERSION 3.13)

add_compile_options(-Wall)

set(PICO_PLATFORM "rp2040")

include(pico-sdk/pico_sdk_init.cmake)
project(smd_pico_controller)
pico_sdk_init()

add_executable(${PROJECT_NAME}
	controller.c
	clocked_read.c
	xop.c
	command_parser.c
	base64.c
	adler32.c
	dumbrle.c
	base.c
	loopback_test.c
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE
	pico_stdlib
	pico_multicore
	hardware_pio
	hardware_dma
	pico_unique_id
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/clocked_read.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/loopback_test.pio)

pico_add_extra_outputs(${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME} PRIVATE PICO_ENTER_USB_BOOT_ON_EXIT=1)
