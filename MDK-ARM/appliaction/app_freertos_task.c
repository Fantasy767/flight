#include "app_freertos_task.h"
#include "LED.h"
#include "SI24R1.h"
#include "comment.h"
#include "led.h"
#include "motor.h"
#include "mpu6050.h"
#include "tim.h"
#include "tp4336.h"

//栈溢出钩子（FreeRTOSConfig.h 里 configCHECK_FOR_STACK_OVERFLOW=2 时会被调用）。
//注意：这个钩子是在**中断/PendSV 上下文**里被调用的，此时栈已经不够用了，
//所以绝对不能在这里调用 printf —— 那只会再溢出一次，把现场彻底搅乱。
//正确做法：把任务名留在全局变量里给调试器看，然后停住，让问题立刻暴露，
//而不是继续跑下去静默踩内存（那样会表现为各种无法解释的怪现象）。
volatile char stack_overflow_task[16]={0};
volatile uint32_t stack_overflow_count=0;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    for(int i=0;i<15 && pcTaskName[i]!='\0';i++)
    {
        stack_overflow_task[i]=pcTaskName[i];
    }
    stack_overflow_count++;
    taskDISABLE_INTERRUPTS();
    for(;;)
    {
        //停在这里。用调试器看 stack_overflow_task / stack_overflow_count 就知道是谁溢出了
    }
}

//内存管理的结构体全部保存在堆，不会自动回收，需要手动释放  
//始终使用同一个结构体不断使用




LED_TypeDef led_bottom={
    .GPIOx=LED1_GPIO_Port,
    .GPIO_Pin=LED1_Pin
};
LED_TypeDef led_top_left={
    .GPIOx=LED3_GPIO_Port,
    .GPIO_Pin=LED3_Pin
};
LED_TypeDef led_top_right={
    .GPIOx=LED4_GPIO_Port,
    .GPIO_Pin=LED4_Pin
};

//表示当前遥控器的连接状态
remote_state_enum remote_state=REMOTE_DISCONNECTED;
fly_state_enum fly_state=FAIL   ;//默认是失控保护状态，防止意外启动

/*int16_t thr;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t off_or_on;//开关机 ,1为关机 0开机
    uint8_t height;*/
 remote_data remo_data={.thr=0, .yaw=500, .pitch=500, .roll=500, .off_or_on=0, .height=0};
//led控制任务
void led_task(void *args);
//栈大小单位是"字"(Cortex-M3 下 1字=4字节)，128字只有512字节。
//这些任务都要调用 printf/newlib 格式化，512字节很容易溢出，所以加大到 384字(1536字节)。
#define led_task_stack_size 384 
#define led_task_priority 1//数值越小，中断优先级越低
#define led_task_delay 50//ms
TaskHandle_t led_task_handle;




  void powtask(void *args);
 #define powtask_stack_size 256
 #define powtask_priority 4//数值越小，中断优先级越低
 TaskHandle_t powtask_handle;

 //飞行电机
 void motor_task(void *args);
 #define motor_task_stack_size 384      
 #define motor_task_priority 3//数值越小，中断优先级越低
 TaskHandle_t motor_task_handle;
 #define motor_task_delay 10//ms，与 PID_PERIOD/IMU dt=0.01s 对齐
//si24r1
 void remote_task(void *args);
#define remote_stack_size 256
 #define remote_priority 2//数值越小，中断优先级越低
 TaskHandle_t remote_handle;
 #define remote_period  6

//打印各任务栈的历史最小剩余量（单位：字，数值越大表示空余越多）。
//接近 0 就说明栈不够，必须继续加大。用实测数据说话，不靠猜。


 //启动rtos
void app_freertos_task(void)
{
    //必须先初始化 MPU6050：芯片上电默认是 SLEEP 模式，传感器不采样，
    //输出寄存器恒为 0，串口就打印一串 0（init 调用被误删时就是这个症状）。
    //放在这里（调度器启动前）调用：HAL_Delay 可靠，也不会有任务并发访问 I2C。
   // mpu6050_init();

    //电源管理任务
    xTaskCreate(powtask, "powtask", powtask_stack_size, NULL, powtask_priority, &powtask_handle);
    //创建飞行任务
    xTaskCreate(motor_task, "motor_task", motor_task_stack_size, NULL, motor_task_priority, &motor_task_handle);
    //创建led任务
    xTaskCreate(led_task, "led_task", led_task_stack_size, NULL, led_task_priority, &led_task_handle);
    //si24r1任务
    xTaskCreate(remote_task, "remote_task", remote_stack_size, NULL, remote_priority, &remote_handle);

    //启动调度器
    vTaskStartScheduler();
}
void powtask(void *args)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    //task启动之后，不断执行的内容
    while (1)
    {
        
        //10s启动一次电源管理
       // vTaskDelayUntil(&xLastWakeTime, 10000 );//延时1000ms，结束cpu占用，不是让cpu空转

       // tp4336_init();//10s启动一次电源管理

        uint32_t res=ulTaskNotifyTake(pdTRUE, 10000);
        if(res!=0)
        {
            //关机
            tp4336_off();
        }
        else
        {
            //无需关机
            tp4336_init();
        }
    }
}
void motor_task(void *args)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    //MPU6050 已经在 app_freertos_task() 里（调度器启动前）初始化过了
   flight_test_init();
    //task启动之后，不断执行的内容
    while (1)
    {
        
        //获取三周加速度和角速度
        flight_get_eular();
        //延时1000ms，结束cpu占用，不是让cpu空转
        app_flight_pid_process();
        
        app_flight_control_motor();
        

        vTaskDelayUntil(&xLastWakeTime, motor_task_delay );
        //飞行电机任务
       
    }
}

void led_task(void *args)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    //task启动之后，不断执行的内容
    while (1)
    {
        if(remote_state==REMOTE_CONNECTED)//如果遥控器连接上了
        {
            
            LED__Start(&led_top_left);
            LED__Start(&led_top_right);
            //点亮上面两个灯
        }
        else
        {
            //没连上下面的打开，上面的关闭
            LED__Start(&led_bottom);
            LED__Stop(&led_top_left);
            LED__Stop(&led_top_right);
        }

        if(fly_state==free_fly)
        {
            //空闲飞行，下面的灯闪烁
            LED__Toggle(&led_bottom);
        }
        else if(fly_state==mormal_fly)
        {
            //正常飞行，左上的灯闪烁
            LED__Toggle(&led_top_left);
        }
        else if(fly_state==high_Fix)
        {
            //高空定点，右上的灯闪烁
            LED__Toggle(&led_top_right);
        }
        else if(fly_state==FAIL)
        {
            //失控保护，全部的灯闪烁
            LED__Toggle(&led_bottom);
            LED__Toggle(&led_top_left);
            LED__Toggle(&led_top_right);
        }
        vTaskDelayUntil(&xLastWakeTime, led_task_delay );//延时1ms，结束cpu占用，不是让cpu空转
    }
}

uint8_t remote_data_base[TX_PLOAD_WIDTH+1]={0};
 void remote_task(void *args)
 {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    //task启动之后，不断执行的内容
    while (1)
    {
       uint8_t res=0;
       res=app_receive_data();
       app_receive_remote_state(res);

       //处理关机命令
       if(remo_data.off_or_on==1)
       {
        // tp4336_off();
        xTaskNotifyGive(powtask_handle);
       }
        
       

       app_processs_flight_state();//根据遥控数据/连接状态更新飞行状态机

       //6ms一次 接收数据
        vTaskDelayUntil(&xLastWakeTime, remote_period);//延时6000ms
        
    }
 } 
