# Guard against double-inclusion (e.g. toolchainFile + include()).
if(DEFINED STM32_GCC_ARM_NONE_EABI_TOOLCHAIN_INCLUDED)
  return()
endif()
set(STM32_GCC_ARM_NONE_EABI_TOOLCHAIN_INCLUDED TRUE)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Avoid try_compile() attempting to run binaries on the host.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc REQUIRED)
find_program(ARM_NONE_EABI_GXX arm-none-eabi-g++ REQUIRED)
find_program(ARM_NONE_EABI_OBJCOPY arm-none-eabi-objcopy)
find_program(ARM_NONE_EABI_SIZE arm-none-eabi-size)

set(CMAKE_C_COMPILER "${ARM_NONE_EABI_GCC}")
set(CMAKE_CXX_COMPILER "${ARM_NONE_EABI_GXX}")
set(CMAKE_ASM_COMPILER "${ARM_NONE_EABI_GCC}")

# Default CPU flags for STM32F446 (Cortex-M4F).
set(STM32_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT "${STM32_CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_CXX_FLAGS_INIT "${STM32_CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_ASM_FLAGS_INIT "${STM32_CPU_FLAGS} -x assembler-with-cpp")

set(CMAKE_EXE_LINKER_FLAGS_INIT "${STM32_CPU_FLAGS} -Wl,--gc-sections")

