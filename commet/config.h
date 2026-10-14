#ifndef CONFIG_H
#define CONFIG_H
#include "main.h"
typedef enum
{
    REMOTE_CONNECTED=0,
    REMOTE_DISCONNECTED=1,
} remote_state_enum;
//飞行状态
typedef enum
{
    free_fly=0,//空闲飞行
    mormal_fly=1,//正常飞行
    high_Fix=2,//高空定点
    FAIL=3,//失控保护
} fly_state_enum;


typedef  struct
{
    int16_t thr;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t off_or_on;//开关机 ,1为关机 0开机
    uint8_t height; //0不切换  1

}remote_data;
typedef enum
{
    free_thr=0,
    max,
    leavel_max,
    min,
    unlock,
} Thr_state;
//mpu6050
typedef struct
{
    int16_t gyro_x;//向右飞为+，，表示横滚角
    int16_t gyro_y;//向前飞转动为+，向后飞转动为-
    int16_t gyro_z;//顺时针转动为-，表示偏航角
}gyro;  //角速度 单位 °/s
typedef struct
{
    int16_t accel_x;//往前+
    int16_t accel_y;//往左为+
    int16_t accel_z;//朝上的加速度为正
}accel;//加速度 单位 g
typedef struct
{
    gyro gyro;
    accel accel;
}mpu6050_data;

//结算得到的欧拉角
typedef struct
{
    float yaw;
    float pitch;
    float roll;
}eular;

#endif /* CONFIG_H */   