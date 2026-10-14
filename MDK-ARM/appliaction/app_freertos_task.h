#ifndef APP_FREERTOS_TASK_H_
#define APP_FREERTOS_TASK_H_
#include"FreeRTOS.h"
#include"task.h"
#include"comment.h"
#include"tp4336.h"
#include"motor.h"
#include"led.h"
#include"config.h"
#include"SI24R1.h"
#include "app_flight.h"
#include "App_receive_data.h"
//启动 rtos de任务
void app_freertos_task(void);


#endif /* APP_FREERTOS_TASK_H_ */

