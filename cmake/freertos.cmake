add_library(freertos INTERFACE IMPORTED)
set(FREERTOS_PATH ${CMAKE_SOURCE_DIR}/Thirdparty/FreeRTOS)

target_sources(freertos INTERFACE
  ${FREERTOS_PATH}/croutine.c
  ${FREERTOS_PATH}/event_groups.c
  ${FREERTOS_PATH}/list.c
  ${FREERTOS_PATH}/queue.c
  ${FREERTOS_PATH}/stream_buffer.c
  ${FREERTOS_PATH}/tasks.c
  ${FREERTOS_PATH}/timers.c
  ${FREERTOS_PATH}/portable/GCC/ARM_CM4F/port.c
  ${FREERTOS_PATH}/portable/MemMang/heap_4.c
)

target_include_directories(freertos INTERFACE
  ${FREERTOS_PATH}/portable/GCC/ARM_CM4F
  ${FREERTOS_PATH}/portable/MemMang
  ${FREERTOS_PATH}/include
)
