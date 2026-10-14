#include"tp4336.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
//启动tp4336模块，防止关机
void tp4336_init(void)
{
    HAL_GPIO_WritePin(powkey_GPIO_Port, powkey_Pin, GPIO_PIN_RESET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(powkey_GPIO_Port, powkey_Pin, GPIO_PIN_SET);
}
void tp4336_off(void)
{
    HAL_GPIO_WritePin(powkey_GPIO_Port, powkey_Pin, GPIO_PIN_RESET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(powkey_GPIO_Port, powkey_Pin, GPIO_PIN_SET);
    vTaskDelay(200);
    HAL_GPIO_WritePin(powkey_GPIO_Port, powkey_Pin, GPIO_PIN_RESET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(powkey_GPIO_Port, powkey_Pin, GPIO_PIN_SET);


}
