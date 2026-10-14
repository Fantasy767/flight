#ifndef __IMU_H
#define __IMU_H

#include "config.h"
#include "math.h"

/* ��ʾ��Ԫ���Ľṹ�� */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

extern float RtA;
extern float Gyro_G;
extern float Gyro_Gr;

void IMU_GetEulerAngle(mpu6050_data *gyroAccel,
                              eular *eulerAngle,
                              float dt);
float IMU_GetNormAccZ(void);

#endif
