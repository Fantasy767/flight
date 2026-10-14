#include "SI24R1.h"
#include "comment.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef WRITE_REG
#undef WRITE_REG
#endif
#define WRITE_REG SI24R1_WRITE_REG

uint8_t  TX_ADDRESS[TX_ADR_WIDTH] = {0x0A,0x01,0x07,0x0E,0x01};  //     һ    ̬   ͵ ַ


/**
 * @brief 通过SPI总线发送一个字节并读取返回值
 * @param byte 要发送的数据
 * @return SPI读取到的响应字节
 */
static uint8_t SPI_RW(uint8_t byte)
{
	uint8_t rxdata=0;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &rxdata, 1, 1000);   
	return rxdata;                        
}





/**
 * @brief 向SI24R1寄存器写入一个字节
 * @param reg 寄存器地址
 * @param value 要写入的数据
 * @return 状态寄存器值
 */
uint8_t SI24R1_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CS_LOW;                 
	status = SPI_RW(reg);				
	SPI_RW(value);
	CS_HIGH;  
	
	return(status);
}


/**
 * @brief 向SI24R1寄存器连续写入一段数据
 * @param reg 寄存器地址
 * @param pBuf 数据缓冲区指针
 * @param size 数据长度
 * @return 状态寄存器值
 */
uint8_t SI24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t size)
{
	uint8_t status,byte_ctr;

  CS_LOW;                                 			
  status = SPI_RW(reg);                          
  for(byte_ctr=0; byte_ctr<size; byte_ctr++)     
    SPI_RW(*pBuf++);
  CS_HIGH;                                    	

  return(status);       
}							  					   


/**
 * @brief 读取SI24R1寄存器的一个字节
 * @param reg 寄存器地址
 * @return 寄存器读取到的值
 */
uint8_t SI24R1_Read_Reg(uint8_t reg)
{
 	uint8_t value;

	CS_LOW;
	SPI_RW(reg);			
	value = SPI_RW(0);
	CS_HIGH;             

	return(value);
}


/**
 * @brief 连续读取SI24R1寄存器中的一段数据
 * @param reg 寄存器地址
 * @param pBuf 用于接收数据的缓冲区指针
 * @param size 数据长度
 * @return 状态寄存器值
 */
uint8_t SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t size)
{
	uint8_t status,byte_ctr;

  CS_LOW;                                        
  status = SPI_RW(reg);                           
  for(byte_ctr=0;byte_ctr<size;byte_ctr++)
    pBuf[byte_ctr] = SPI_RW(0);                   //ȡݣֽǰ
  CS_HIGH;                                        

  return(status);    
}



/**
 * @brief 配置SI24R1为接收模式
 */
void SI24R1_RX_Mode(void)
{
	CE_LOW;
	SI24R1_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);	//      豸    ͨ  0ʹ úͷ    豸  ͬ ķ  ͵ ַ
	SI24R1_Write_Reg(WRITE_REG + EN_AA, 0x01);               						// ʹ ܽ   ͨ  0 Զ Ӧ  
	SI24R1_Write_Reg(WRITE_REG + EN_RXADDR, 0x01);           						// ʹ ܽ   ͨ  0
	SI24R1_Write_Reg(WRITE_REG + RF_CH, CHANNEL);                 						// ѡ    Ƶͨ  0x40
	SI24R1_Write_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  						//     ͨ  0ѡ  ͷ   ͨ    ͬ  Ч   ݿ   
	SI24R1_Write_Reg(WRITE_REG + RF_SETUP, 0x06);            						//    ݴ     2Mbps     书  7dBm
	SI24R1_Write_Reg(WRITE_REG + CONFIG, 0x0f);              						// CRCʹ ܣ 16λCRCУ 飬 ϵ磬    ģʽ
	SI24R1_Write_Reg(WRITE_REG + STATUS, 0xff);  												//      е  жϱ ־λ
	CE_HIGH;                                            									//CE豸
}						



/**
 * @brief 配置SI24R1为发送模式
 */
void SI24R1_TX_Mode(void)
{
	CE_LOW;
	SI24R1_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // д 뷢 ͵ ַ
	SI24R1_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // Ϊ  Ӧ      豸      ͨ  0  ַ ͷ  ͵ ַ  ͬ

	SI24R1_Write_Reg(WRITE_REG + EN_AA, 0x01);       											// ʹ ܽ   ͨ  0 Զ Ӧ  
	SI24R1_Write_Reg(WRITE_REG + EN_RXADDR, 0x01);   											// ʹ ܽ   ͨ  0
	SI24R1_Write_Reg(WRITE_REG + SETUP_RETR, 0x0a);  											//  Զ  ط   ʱ ȴ 250us+86us   Զ  ط 10  
	SI24R1_Write_Reg(WRITE_REG + RF_CH, CHANNEL);         											// ѡ    Ƶͨ  0x40
	SI24R1_Write_Reg(WRITE_REG + RF_SETUP, 0x06);    											//    ݴ     2Mbps     书  7dBm
	SI24R1_Write_Reg(WRITE_REG + CONFIG, 0x0e);      											// CRCʹ ܣ 16λCRCУ 飬 ϵ 
	CE_HIGH;                                            										//CE豸
}



/**
 * @brief 接收一帧数据包
 * @param rxbuf 接收缓冲区指针
 * @return 0表示收到数据，1表示未收到数据
 */
uint8_t SI24R1_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;
	state = SI24R1_Read_Reg(STATUS);  			                 //  ȡ״̬ Ĵ     ֵ    	  
	SI24R1_Write_Reg(WRITE_REG+STATUS,state);               //   RX_DS жϱ ־

	if(state & RX_DR)								                           //   յ     
	{
		SI24R1_Read_Buf(RD_RX_PLOAD,rxbuf,TX_PLOAD_WIDTH);     //  ȡ    
		SI24R1_Write_Reg(FLUSH_RX,0xff);					              //   RX FIFO Ĵ   
		return 0; 
	}	   
	return 1;                                                   //û յ  κ     
}



/**
 * @brief 发送一帧数据包
 * @param txbuf 发送缓冲区指针
 * @return 0表示发送成功，1表示发送失败
 */
uint8_t SI24R1_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
	CE_LOW;
  SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	   
	CE_HIGH;
	
	//没有使用中断判断是否发送完成，使用轮询
	//while(IRQ == 1);																				  
	state = SI24R1_Read_Reg(STATUS);  	
	while ((state & TX_DS) == 0 && (state & MAX_RT) == 0)  //等待发送完成
	{
		state = SI24R1_Read_Reg(STATUS);  	
		vTaskDelay(1);
	}					    
	SI24R1_Write_Reg(WRITE_REG+STATUS, state); 								
	if(state&MAX_RT)																			 
	{
		SI24R1_Write_Reg(FLUSH_TX,0xff);										
		return 1; 
	}
	if(state&TX_DS)																			     
	{
		return 0;
	}
	return 1;																						  //ʧ
}

uint8_t si24r1_rxbuf[5];
uint8_t si24r1_check()
{
	 
	SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, si24r1_rxbuf, TX_ADR_WIDTH);	//x写入一个数据
	SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // д 뷢 ͵ ַ
	

	//读取一个数据
	SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, si24r1_rxbuf, TX_ADR_WIDTH);

	//debug_printf("buff:%02x %02x %02x %02x %02x\n",
	//si24r1_rxbuf[0],si24r1_rxbuf[1],si24r1_rxbuf[2],si24r1_rxbuf[3],si24r1_rxbuf[4]);
	for(uint8_t i=0;i<TX_ADR_WIDTH;++i)
	{
		if(si24r1_rxbuf[i]!=TX_ADDRESS[i])
		{
			//debug_printf("si24r1 check fail\n");
			return 1;
		}
		
		
	}
	return 0;
}

uint8_t si24r1_rxbuf[5];
/**
 * @brief 初始化SI24R1的控制引脚状态
 */
void SI24R1_Init(void)
{
	HAL_Delay(200);//上电延时
   while(si24r1_check()==1)
   {
	 HAL_Delay(10);
   }
   //设置默认状态是接收  每次发送的时候切换成发送状态
   SI24R1_RX_Mode();
   debug_printf("si24r1 init success\r\n");
}