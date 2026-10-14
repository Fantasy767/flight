#ifndef __APP_RECIVE_DATA_H__
#define __APP_RECIVE_DATA_H__
#include "SI24R1.h"
#include "config.h"
#include <stdint.h>
#define FIRST_HEAD_CHECK 'w'
#define SECOND_HEAD_CHECK 'z'
#define THIRD_HEAD_CHECK 'y'

uint8_t app_receive_data();
void app_receive_remote_state(uint8_t state);
void app_processs_flight_state();
#endif