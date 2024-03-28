
cmake_minimum_required(VERSION 3.16)

# Path to toolchain file. This one has to be before 'project()' below
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake)

project(F429_FreeRTOS_LCD)
set(EXECUTABLE ${PROJECT_NAME}.elf)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/STM32F429ZITX_FLASH.ld)

set(CMAKE_C_FLAGS_DEBUG
  "${CMAKE_C_FLAGS_DEBUG} -O0 -ggdb"
  CACHE INTERNAL " C compiler flags")
set(CMAKE_C_FLAGS_RELEASE
  "${CMAKE_C_FLAGS_RELEASE} -O3"
  CACHE INTERNAL " C compiler flags")
set(CMAKE_C_FLAGS_RELWITHDEBINFO
  "${CMAKE_C_FLAGS_RELWITHDEBINFO} -O3 -ggdb"
  CACHE INTERNAL " C compiler flags")
set(CMAKE_C_FLAGS_MINSIZEREL
  "${CMAKE_C_FLAGS_MINSIZEREL} -O3"
  CACHE INTERNAL " C compiler flags")


add_compile_options(
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard

  -fdata-sections
  -ffunction-sections

  -DUSE_HAL_DRIVER
  -DSTM32F429xx
  -Wall
)

# set(DEFINE_COMPILE_FLAGS
#   -mcpu=cortex-m4
#   -mthumb
#   -mfpu=fpv4-sp-d16
#   -mfloat-abi=hard

#   -fdata-sections
#   -ffunction-sections

#   -DUSE_HAL_DRIVER
#   -DSTM32F429xx
#   -Wall
#   -O0
#   -g3
# )

enable_language(C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

include(cmake/freertos.cmake)

set(FREERTOS_DIR
  ${FREERTOS_PATH}/include
  ${FREERTOS_PATH}/portable/GCC/ARM_CM4F
  CACHE PATH "Path to FreeRTOS include directory"
  FORCE
)

set(STM32F4xx_DIR
  ${CMAKE_SOURCE_DIR}/Core/Inc
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
  CACHE PATH "Path to CMSIS of STM32F4xx"
  FORCE
)

# for lvgl
set(LV_CONF_BUILD_DISABLE_THORVG_INTERNAL ON)
set(LV_CONF_BUILD_DISABLE_EXAMPLES ON)
# set(LV_CONF_BUILD_DISABLE_DEMOS ON)
set(LV_CONF_PATH
  ${CMAKE_SOURCE_DIR}/lv_conf.h
  CACHE STRING "" FORCE)

add_subdirectory(Thirdparty/lvgl)

set(STM32F4HALDIR "Drivers/STM32F4xx_HAL_Driver/Src")
FILE(GLOB STM32F4_HAL_DRIVER "${STM32F4HALDIR}/*.c")

set(SRC_FILES
  Core/Src/main.c
  Core/Src/stm32f4xx_hal_msp.c
  Core/Src/stm32f4xx_it.c
  Core/Src/syscalls.c
  Core/Src/sysmem.c
  Core/Src/system_stm32f4xx.c
  Core/Startup/startup_stm32f429zitx.s
  ${STM32F4_HAL_DRIVER}
  Core/Src/freertos_hook.c
  Core/Src/gcc_printf_port.c
  Core/Src/freertos_test.c
  Core/Src/lvgl_port.c
)

# Build the executable based on the source files
add_executable(${EXECUTABLE} ${SRC_FILES})

add_dependencies(${EXECUTABLE}   lvgl::demos)

target_link_libraries(${EXECUTABLE}
  freertos
  lvgl::lvgl
  lvgl::demos
)

# List of includ directories
target_include_directories(${EXECUTABLE} PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${STM32F4xx_DIR}
)

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
        ${DEFINE_COMPILE_FLAGS}
        )

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
  -T${LINKER_FILE}
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  -specs=nano.specs
  -specs=nosys.specs
  -lc
  -lm
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -Wl,--gc-sections
  -Xlinker -print-memory-usage -Xlinker
)

# Optional: Print executable size as part of the post build process
add_custom_command(TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE})

# Optional: Create hex, bin and S-Record files after the build
add_custom_command(TARGET ${EXECUTABLE}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)
