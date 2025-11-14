#ifndef __MYI2C_H__
#define __MYI2C_H__
/*
 * 基础特性概览（软件 I2C）
 * - 职责：基于 GPIO 的位操作实现 I2C 主机，适配简单外设。
 * - 引脚：默认 PB10(SCL)、PB11(SDA)，可通过宏修改。
 * - 时序：微秒级延时实现 START/STOP/ACK/NACK；速率受 CPU 影响。
 * - 场景：在硬件 I2C 受限或需多总线时作为备选；不推荐高带宽传感器。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "gpio.h"

#define CPU_FREQUENCY_MHZ   0 //4
#define MYI2C_SCL_PIN        GPIO_PIN_10
#define MYI2C_SCL_PORT        GPIOB
#define MYI2C_SDA_PIN            GPIO_PIN_11
#define MYI2C_SDA_PORT        GPIOB

#define SDA_Dout_LOW()                          HAL_GPIO_WritePin(MYI2C_SDA_PORT,MYI2C_SDA_PIN,GPIO_PIN_RESET)
#define SDA_Dout_HIGH()                         HAL_GPIO_WritePin(MYI2C_SDA_PORT,MYI2C_SDA_PIN,GPIO_PIN_SET)
#define SDA_Data_IN()                           HAL_GPIO_ReadPin(MYI2C_SDA_PORT,MYI2C_SDA_PIN)
#define SCL_Dout_LOW()                          HAL_GPIO_WritePin(MYI2C_SCL_PORT,MYI2C_SCL_PIN,GPIO_PIN_RESET)
#define SCL_Dout_HIGH()                         HAL_GPIO_WritePin(MYI2C_SCL_PORT,MYI2C_SCL_PIN,GPIO_PIN_SET)
#define SCL_Data_IN()                           HAL_GPIO_ReadPin(MYI2C_SCL_PORT,MYI2C_SCL_PIN)
#define SDA_Write(XX)                           HAL_GPIO_WritePin(MYI2C_SDA_PORT,MYI2C_SDA_PIN,(XX?GPIO_PIN_SET:GPIO_PIN_RESET))

#define I2C_SOFT                ((I2C_TypeDef *) 0x00000000UL)
extern I2C_HandleTypeDef hi2c0;

void Soft_I2C_Init(void);

void Soft_I2C_Start(void);

void Soft_I2C_Stop(void);

void Soft_I2C_Send_Byte(uint8_t txd);

uint8_t Soft_I2C_Read_Byte(uint8_t ack);

void Soft_I2C_NAck(void);

void Soft_I2C_Ack(void);

uint8_t Soft_I2C_Wait_Ack(void);


void delay_xus(__IO uint32_t nTime);

void SOFT_I2C_Master_Transmit(uint8_t daddr, uint8_t *buff, uint8_t len);


#define Delay_us(xx)  delay_xus(xx)

#ifdef __cplusplus
}
#endif
#endif