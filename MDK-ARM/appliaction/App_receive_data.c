#include "App_receive_data.h"
#include "SI24R1.h"
#include "config.h"
#include <string.h>
extern remote_data remo_data;
uint8_t rx_buff[TX_PLOAD_WIDTH]={0};
extern remote_state_enum remote_state;
#define MAX_ERROR 5
uint8_t MAX_retry_count=0;
//油门状态
Thr_state thr_state=free_thr;
extern fly_state_enum fly_state;


//if return0 校验通过，是正常的数据，elseif  return1, 是非法数据
uint8_t app_receive_data()
{
    memset(rx_buff,0,TX_PLOAD_WIDTH);
    SI24R1_RxPacket(rx_buff);
    if(strlen(rx_buff)==0)
    {
        return 1;
    }
    //枕头校验
    if(rx_buff[0]!=FIRST_HEAD_CHECK||rx_buff[1]!=SECOND_HEAD_CHECK
    ||rx_buff[2]!=THIRD_HEAD_CHECK)
    {
        return 1;
    }

    //帧尾校验
    uint32_t sum=0;
    uint32_t sum_receive=0;
       
    for(uint8_t i=0;i<13;++i)
    {
        sum+=rx_buff[i];
    }
    //高位在前
    sum_receive=rx_buff[13]<<24|rx_buff[14]<<16|rx_buff[15]<<8|rx_buff[16];
    if(sum!=sum_receive)
    {
        return 1;
    }
    //成功之后保存数据
    remo_data.thr=rx_buff[3]<<8|rx_buff[4];
    remo_data.yaw=rx_buff[5]<<8|rx_buff[6];
    remo_data.pitch=rx_buff[7]<<8|rx_buff[8];
    remo_data.roll=rx_buff[9]<<8|rx_buff[10];
    remo_data.off_or_on=rx_buff[11];
    remo_data.height=rx_buff[12];

    debug_printf(":%d,%d,%d,%d,%d,%d\n",remo_data.thr,remo_data.yaw,remo_data.pitch,
        remo_data.roll,remo_data.off_or_on,remo_data.height);
    return 0;
}
//处理连接状态
//state：上一次接收数据的返回值
void app_receive_remote_state(uint8_t state)
{
    if(state==0)
    {
        remote_state=REMOTE_CONNECTED;
        MAX_retry_count=0;
        //连接成功
    }
    else if(state==1)
    {

        //连接失败
        MAX_retry_count++;
        if(MAX_retry_count>=MAX_ERROR)
        {
            remote_state=REMOTE_DISCONNECTED;
            MAX_retry_count=0;
        }
    }
}
uint32_t enter_time=0;
uint32_t min_leave_time=0;
static uint8_t app_process_unlock()//解锁逻辑
{
    /*free_thr=0,
    max,
    leavel_max,
    min,
    unlock,*/
    switch (thr_state)
    {
        case free_thr:
            if(remo_data.thr>=900)
            {
                thr_state=max;
                enter_time=xTaskGetTickCount();
            }
            break;
        case max:
            //持续的事件应该是离开事件减去进入进入时间
            if(remo_data.thr<900)
            {
                if(xTaskGetTickCount()-enter_time>1000)
                {
                    //油门保持最高状态超过1s,进入leavel_max
                    thr_state=leavel_max;
                }
                else
                {
                    //油门保持最高状态小于1s
                    thr_state=free_thr;
                }
            }
            break;
        case leavel_max:
            if(remo_data.thr<100)
            {
                thr_state=min;
                min_leave_time=xTaskGetTickCount();
            }
            break;
        case min:
            if(xTaskGetTickCount()-min_leave_time<1000)
            {
                if(remo_data.thr>100)
                {
                    thr_state=free_thr;
                }
            }
            else
            {
                thr_state=unlock;   
            }
            break;
        case unlock:

            break;
        default:
            break;
    }
    if(thr_state==unlock)
    {
        return 0;
    }
    return 1;
}
void app_processs_flight_state()
{
    //轮询调用判断当前的状态
   // free_fly=0,//空闲飞行
   // mormal_fly=1,//正常飞行
   // high_Fix=2,//高空定点
   // FAIL=3,//失控保护
    switch(fly_state)
    {
        case free_fly:
            if(app_process_unlock()==0)
            {
                fly_state=mormal_fly;
                thr_state=free_thr;
            }
            break;

        case mormal_fly:
            if(remo_data.height==1)
            {
                fly_state=high_Fix;
                remo_data.height=0;
            }
            //故障失联
            if(remote_state==REMOTE_DISCONNECTED)
            {
                fly_state=FAIL;
            }
            break;

        case high_Fix:
        //取消 定高
            if(remo_data.height==1)
            {
                fly_state=mormal_fly;
                remo_data.height=0;
            }
            //故障失联
            if(remote_state==REMOTE_DISCONNECTED)
            {
                fly_state=FAIL;
            }
            break;

        case FAIL:
            //失恋故障  电机需缓停止
            vTaskDelay(1);
            fly_state=free_fly;
            break;

         default:
            break;
    }

}