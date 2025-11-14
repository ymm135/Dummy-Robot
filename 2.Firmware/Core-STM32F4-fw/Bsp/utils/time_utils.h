#ifndef REF_STM32F4_TIME_UTILS_H
#define REF_STM32F4_TIME_UTILS_H
/*
 * 基础特性概览（时间工具）
 * - 职责：提供微秒/毫秒时间戳与短延迟，基于 SysTick/HAL。
 * - micros：结合 SysTick COUNTFLAG 校准子毫秒进度，避免遗漏溢出。
 * - delayMicroseconds：简易忙等，适合短时序补偿；勿用于长阻塞。
 * - 注意：避免在高优先级任务/ISR 中长时间调用，防止抖动。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

uint32_t micros(void);
uint32_t millis(void);
void delayMicroseconds(uint32_t us);

#ifdef __cplusplus
}
#endif
#endif //REF_STM32F4_TIME_UTILS_H
