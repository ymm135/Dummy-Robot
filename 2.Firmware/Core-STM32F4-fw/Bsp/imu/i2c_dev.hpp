// 迁移自 Arduino 版本，适配 STM32 HAL I2C 驱动
/*
 * 基础特性概览（I2C 设备读写）
 * - 职责：基于 STM32 HAL 的寄存器级 I2C 读写 API。
 * - 超时：默认 1000ms，可覆盖；建议在任务层调用避免阻塞控制环。
 * - 语义：写函数多为读-改-写，降低对寄存器的破坏性操作风险。
 * - 用法：先调用 I2Cdev_init 绑定总线；随后按需使用各 Read/Write API。
 */


#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "i2c.h"

#define true 1
#define false 0

// 读操作默认超时 1000ms (可通过 I2Cdev_readTimeout 修改)
#define I2CDEV_DEFAULT_READ_TIMEOUT     1000

// 初始化：绑定一个 HAL I2C 句柄，后续读写均使用该总线。
void I2Cdev_init(I2C_HandleTypeDef *hi2c);

// 读/写 API 说明：
// - readBit(s)/readByte(s)/readWord(s)：从设备寄存器读取位/字节/字，带可选超时。
// - writeBit(s)/writeByte(s)/writeWord(s)：写入位/字节/字，内部会先读-改-写。
uint8_t I2Cdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data,
                       uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT);
uint8_t I2Cdev_readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data,
                        uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT);
uint8_t
I2Cdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data,
                uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT);
uint8_t
I2Cdev_readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data,
                 uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT);
uint8_t
I2Cdev_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT);
uint8_t
I2Cdev_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT);
uint8_t I2Cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data,
                         uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT);
uint8_t I2Cdev_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data,
                         uint16_t timeout = I2CDEV_DEFAULT_READ_TIMEOUT);

uint16_t I2Cdev_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
uint16_t I2Cdev_writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
uint16_t I2Cdev_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
uint16_t I2Cdev_writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
uint16_t I2Cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
uint16_t I2Cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
uint16_t I2Cdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
uint16_t I2Cdev_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

#endif /* _I2CDEV_H_ */

