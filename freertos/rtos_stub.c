#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

BaseType_t xTaskGetSchedulerState(void)
{
    return taskSCHEDULER_NOT_STARTED;
}

void xPortSysTickHandler(void)
{
}

void vPortEnterCritical(void)
{
}

void vPortExitCritical(void)
{
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    (void)xExpectedIdleTime;
}

void vTaskSwitchContext(void)
{
}

BaseType_t xTaskIncrementTick(void)
{
    return pdFALSE;
}
