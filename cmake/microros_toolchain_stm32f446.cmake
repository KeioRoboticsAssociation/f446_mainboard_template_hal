set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(ARM_NONE_EABI_GCC arm-none-eabi-gcc REQUIRED)
find_program(ARM_NONE_EABI_GXX arm-none-eabi-g++ REQUIRED)
find_program(ARM_NONE_EABI_AR arm-none-eabi-ar REQUIRED)
find_program(ARM_NONE_EABI_RANLIB arm-none-eabi-ranlib REQUIRED)

set(CMAKE_C_COMPILER "${ARM_NONE_EABI_GCC}")
set(CMAKE_CXX_COMPILER "${ARM_NONE_EABI_GXX}")
set(CMAKE_AR "${ARM_NONE_EABI_AR}")
set(CMAKE_RANLIB "${ARM_NONE_EABI_RANLIB}")

set(STM32_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(COMMON_FLAGS "${STM32_CPU_FLAGS} -ffunction-sections -fdata-sections -fno-common")

set(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu11 -DCLOCK_MONOTONIC=0")
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -fno-rtti -fno-exceptions -std=gnu++14 -DCLOCK_MONOTONIC=0")

