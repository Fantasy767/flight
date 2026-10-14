#include "comment.h"
#include "stm32f1xx_hal_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
//#include "stm32f1xx_hal.h"

int __io_putchar(int ch)
{
    uint8_t byte = (uint8_t)ch;

    HAL_UART_Transmit(&huart2, &byte, 1, 1000);
    return ch;
}

int fputc(int c, FILE *f)
{
    (void)f;
    __io_putchar(c);
    return c;
}
/*
int _write(int file, char *ptr, int len)
{
    (void)file;

    taskENTER_CRITICAL();
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 1000);
    taskEXIT_CRITICAL();

    return len;
}*/