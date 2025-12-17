cmake_minimum_required(VERSION 3.22)

add_library(stm32cubemx INTERFACE)

file(GLOB CUBE_CORE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/Core/Src/*.c"
)

file(GLOB HAL_DRIVER_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c"
)
list(FILTER HAL_DRIVER_SOURCES EXCLUDE REGEX ".*_template\\.c$")

target_sources(stm32cubemx INTERFACE
  ${CUBE_CORE_SOURCES}
  ${HAL_DRIVER_SOURCES}
  "${CMAKE_SOURCE_DIR}/startup_stm32f446xx.s"
)

target_include_directories(stm32cubemx INTERFACE
  "${CMAKE_SOURCE_DIR}/Core/Inc"
  "${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc"
  "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include"
  "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
)

target_compile_definitions(stm32cubemx INTERFACE
  USE_HAL_DRIVER
  STM32F446xx
  HAL_TIM_MODULE_ENABLED
)

set(STM32_LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F446RETx_FLASH.ld")
target_link_options(stm32cubemx INTERFACE
  "-T${STM32_LINKER_SCRIPT}"
  "-Wl,-Map=${CMAKE_PROJECT_NAME}.map"
  "-Wl,--cref"
  "-Wl,--gc-sections"
)
