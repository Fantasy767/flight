#include "app_flight.h"
#include "comment.h"
#include "filter.h"
#include <math.h>
mpu6050_data gy_ac_all_data={0};
eular eular_angle={0};
gyro gyro_last_data={0};
float gyro_z_sum=0;
//pid
PID pid_pitch={.kp=1.00,.ki=0.00,.kd=0.00};
//y轴酵素单独
PID pid_gyro_y={.kp=2.00,.ki=0.00,.kd=0.00};
extern remote_data remo_data;
extern fly_state_enum fly_state;
//飞行电机
motor_struct left_top_motor={
    .tim=&htim3,
    .channel=TIM_CHANNEL_1,
    .speed=400
};
motor_struct right_top_motor={
    .tim=&htim2,
    .channel=TIM_CHANNEL_2,
    .speed=400  
};
motor_struct left_bottom_motor={
    .tim=&htim4,
    .channel=TIM_CHANNEL_4,
    .speed=400
};
motor_struct right_bottom_motor={
    .tim=&htim1,
    .channel=TIM_CHANNEL_3,
    .speed=400
};
void flight_test_init(void)
{
   mpu6050_init();
   motor_start(&left_top_motor);
   motor_start(&right_top_motor);
   motor_start(&left_bottom_motor);
   motor_start(&right_bottom_motor);
}
void flight_get_eular()
{
    //使用 mpu6050的得到六轴数据
    mpu6050_get_alldata(&gy_ac_all_data);
        
       //角速度低通滤波 需要堆及时性要求高
       //堆准确性要求不高 但 要求计算迅速
      gy_ac_all_data.gyro.gyro_x = Filter_LowPass(gy_ac_all_data.gyro.gyro_x, gyro_last_data.gyro_x);
      gy_ac_all_data.gyro.gyro_y = Filter_LowPass(gy_ac_all_data.gyro.gyro_y, gyro_last_data.gyro_y);
      gy_ac_all_data.gyro.gyro_z = Filter_LowPass(gy_ac_all_data.gyro.gyro_z, gyro_last_data.gyro_z);
      
      //更新最后的陀螺仪数据
      gyro_last_data = gy_ac_all_data.gyro;
      gy_ac_all_data.accel.accel_x=Filter_KalmanFilter(&kfs[0], gy_ac_all_data.accel.accel_x);
      gy_ac_all_data.accel.accel_y=Filter_KalmanFilter(&kfs[1], gy_ac_all_data.accel.accel_y);
      gy_ac_all_data.accel.accel_z=Filter_KalmanFilter(&kfs[2], gy_ac_all_data.accel.accel_z);
      //  debug_printf("%d,%d,%d\n",gy_ac_all_data.gyro.gyro_x,
     // gy_ac_all_data.gyro.gyro_y,gy_ac_all_data.gyro.gyro_z);
        //debug_printf("%d,%d,%d\n",gy_ac_all_data.accel.accel_x,
       // gy_ac_all_data.accel.accel_y,gy_ac_all_data.accel.accel_z);

      /*
        //互补结算求出欧拉角 使用加速度  只有俯仰角和横滚角
        eular_angle.pitch=atan2(gy_ac_all_data.accel.accel_x*1.0,gy_ac_all_data.accel.accel_z)/3.14*180;
        eular_angle.roll=atan2(gy_ac_all_data.accel.accel_y*1.0,gy_ac_all_data.accel.accel_z)/3.14*180;

        //偏航角
        //gyro是原始值(LSB)，2000/32768=1/16.4 换算成°/s
        //0.0064是实测的循环周期(6.4ms)，原写0.006会让角度小7%
        gyro_z_sum+=(gy_ac_all_data.gyro.gyro_z*2000.0/32768.0)*0.0064;
        eular_angle.yaw=gyro_z_sum;*/
        IMU_GetEulerAngle(&gy_ac_all_data,&eular_angle,0.01);
       // debug_printf("%d,%d,%d\n",(int)eular_angle.pitch,(int)eular_angle.roll,(int)eular_angle.yaw);


}
void app_flight_pid_process()
{
   //俯仰角pid
   //需要赋值目标值和测量值
   //外环目标角度-->如果平稳飞行，值=0， 或者遥控器的值
   //范围是10度
   pid_pitch.target=(remo_data.pitch-500)/50;
   //内环测量值，当前俯仰角
   pid_pitch.measure=eular_angle.pitch;
   //外环测量值，当前角速度
   pid_gyro_y.measure=gy_ac_all_data.gyro.gyro_y*2000.0/32768.0;
   //pid计算
   pid_chain(&pid_pitch,&pid_gyro_y);

   //debug_printf(":%d,%d\n", (int)pid_gyro_y.err, (int)pid_gyro_y.out);
   
}
int16_t limit(int16_t value,int16_t min,int16_t max)
{
    if(value>max)
    {
        return max;
    }
    else if(value<min)
    {
        return min;
    }
    return value;
}
void app_flight_control_motor()
{
  /* free_fly=0,//空闲飞行
    mormal_fly=1,//正常飞行
    high_Fix=2,//高空定点
    FAIL=3,//失控保护*/
    //判断飞行状态
    switch (fly_state)
    {
      case free_fly:
      left_top_motor.speed=200;
      right_top_motor.speed=200;
      left_bottom_motor.speed=200;
      right_bottom_motor.speed=200;
        break;
      case mormal_fly:
      left_top_motor.speed=remo_data.thr+ pid_gyro_y.out;
      right_top_motor.speed=remo_data.thr+ pid_gyro_y.out;
      left_bottom_motor.speed=remo_data.thr- pid_gyro_y.out;
      right_bottom_motor.speed=remo_data.thr-pid_gyro_y.out;
        break;
      case high_Fix:
        break;
      case FAIL:
        break;
    }
    left_top_motor.speed = limit(left_top_motor.speed,0,600);
    right_top_motor.speed = limit(right_top_motor.speed,0,600);   
    left_bottom_motor.speed = limit(left_bottom_motor.speed,0,600);
    right_bottom_motor.speed = limit(right_bottom_motor.speed,0,600);

    //安全限制  油门<50，速度全部=0
    if (remo_data.thr < 50)
    {
        left_top_motor.speed = 0;
        right_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_bottom_motor.speed = 0;
    }

    motor_speed_set(&left_top_motor);
    motor_speed_set(&right_top_motor);
    motor_speed_set(&left_bottom_motor);
    motor_speed_set(&right_bottom_motor);

}
