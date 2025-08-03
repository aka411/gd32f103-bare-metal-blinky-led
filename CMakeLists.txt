#CmakeList for blinky Led

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchain.cmake")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(BlinkyLed C CXX ASM)


add_subdirectory("${CMAKE_SOURCE_DIR}/dep/gd32f10x-minimal-cpp-drivers")



set(APP_SRCS

src/main.cpp
src/utils.cpp

startup/system_gd32f10x.cpp
startup/startup_gd32f10x_md.s

)


add_executable(${PROJECT_NAME}.elf

${APP_SRCS}
)

set_target_properties(${PROJECT_NAME}.elf PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED TRUE
    CXX_EXTENSIONS OFF
)


target_include_directories(${PROJECT_NAME}.elf PRIVATE

${CMAKE_SOURCE_DIR}/include


)


target_link_libraries(${PROJECT_NAME}.elf PRIVATE gd32f10x_drivers)


set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/ld/gd32f103c8t6.ld)


# Check if the linker script exists and print for debugging
if(NOT EXISTS "${LINKER_SCRIPT}")
    message(FATAL_ERROR "Linker script not found at: ${LINKER_SCRIPT}")
else()
    message(STATUS "Using linker script: ${LINKER_SCRIPT}")
endif()


target_link_options(${PROJECT_NAME}.elf PRIVATE

"-T${LINKER_SCRIPT}"

"-nostdlib"

"--specs=nosys.specs"

"-Wl,--gc-sections"
)


# Optional:  post-build steps for generating binary/hex files and size info
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating .bin, .hex files and size info"
)
