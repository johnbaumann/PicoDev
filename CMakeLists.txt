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
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PicoDev C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(PicoDev
    src/booty.c
    src/comms.c
    src/payload.s
    src/PicoDev.c
    src/usb-descriptors.c
)

pico_set_program_name(PicoDev "PicoDev")
pico_set_program_version(PicoDev "0.1")

# Generate PIO header
pico_generate_pio_header(PicoDev
    ${CMAKE_CURRENT_LIST_DIR}/src/booty.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/comms.pio
)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PicoDev 0)
pico_enable_stdio_usb(PicoDev 0)

# Add the standard library to the build
target_link_libraries(PicoDev
        pico_stdlib
)

# Add the standard include files to the build
target_include_directories(PicoDev PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ./src
        ${PICO_TINYUSB_PATH}/src
)

# Add any user requested libraries
target_link_libraries(PicoDev 
        hardware_dma
        hardware_pio
        pico_multicore
        pico_unique_id
        tinyusb_device
)

pico_add_extra_outputs(PicoDev)

