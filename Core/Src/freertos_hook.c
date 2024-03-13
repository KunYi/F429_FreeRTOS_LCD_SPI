#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  if task overflow, it will run here.
  * @param  [IN]task handle.
  * @param  [IN]task name string pointer.
  * @retval None
  */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
     while(1)
     {
         //printf("task %s is stack overflow. \r\n", pcTaskName);
         vTaskDelay(500);
     }
}
