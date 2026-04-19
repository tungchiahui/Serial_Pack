#include "led_task.h"
#include "cmsis_os.h"

extern "C" void led_task(void *argument)
{
    for(;;)
    {
        HAL_GPIO_TogglePin(BSP_LED_GPIO_Port, BSP_LED_Pin);
        osDelay((500U * osKernelGetTickFreq()) / 1000U);  // 延时 500ms
    }
}

