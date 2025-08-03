#toolchain.cmake for blinky Led

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)


# Check if the environment variable is set
if(NOT DEFINED ENV{ARM_GCC_TOOLCHAIN_PATH})
    message(FATAL_ERROR "Environment variable ARM_GCC_TOOLCHAIN_PATH is not set. Please set it to the 'bin' directory of your GNU Arm Embedded Toolchain installation.")
endif()
file(TO_CMAKE_PATH $ENV{ARM_GCC_TOOLCHAIN_PATH} ARM_GCC_TOOLCHAIN_PATH)
# Use the environment variable
set(TOOLCHAIN_PREFIX "${ARM_GCC_TOOLCHAIN_PATH}/arm-none-eabi-")

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc.exe)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++.exe)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc.exe) # This is now the full path arm-none-eabi-gcc
set(CMAKE_OBJCOPY      ${TOOLCHAIN_PREFIX}objcopy.exe)
set(CMAKE_SIZE         ${TOOLCHAIN_PREFIX}size.exe)
set(CMAKE_AR           ${TOOLCHAIN_PREFIX}ar.exe)
set(CMAKE_RANLIB       ${TOOLCHAIN_PREFIX}ranlib.exe)
set(CMAKE_NM           ${TOOLCHAIN_PREFIX}nm.exe)
set(CMAKE_OBJDUMP      ${TOOLCHAIN_PREFIX}objdump.exe)


# --- Compiler Flags (Common for Cortex-M3) ---
# Processor-specific flags
set(COMMON_FLAGS "-mcpu=cortex-m3 -mthumb") # Thumb mode is crucial for Cortex-M

# Standard C/C++ flags for embedded
set(COMMON_FLAGS "${COMMON_FLAGS} -Wall -Wextra -Wpedantic") # Warnings
set(COMMON_FLAGS "${COMMON_FLAGS} -fmessage-length=0") # No line breaks in error messages

# Optimization level (e.g., -Os for size optimization)
set(CMAKE_C_FLAGS   "${COMMON_FLAGS}  -Os -fno-builtin   ")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS}  -Os  -fno-builtin -fno-rtti -fno-exceptions ")
