#ifndef __APP_FLIGHT__
#define __APP_FLIGHT__
#include "mpu6050.h"
#include "filter.h"
#include "math.h"
#include "imu.h"
#include "pid.h"
#include "motor.h"
void flight_test_init(void);
void flight_get_eular(void);
void app_flight_pid_process(void);
void app_flight_control_motor(void);
#endif