#ifndef REF_STM32F4_TIMER_HPP
#define REF_STM32F4_TIMER_HPP
/*
 * 基础特性概览（轻量定时器）
 * - 职责：对 HAL TIM 做轻量封装，简化周期定时与回调注册。
 * - 频率计算：根据 APB 域时钟计算 PSC/ARR，默认 100Hz。
 * - 回调模型：在更新中断中调用用户回调；与 FreeRTOS 控制环协作。
 * - 适用：TIM7/10/11/13/14 等基础定时器。
 */

#include <cmath>
#include "tim.h"

typedef void (*TimerCallback_t)();

class Timer
{
private:
    TIM_HandleTypeDef *htim;
    uint32_t freq;
    uint16_t PSC = 83;
    uint16_t ARR = 9999;

    void CalcRegister(uint32_t _freqHz);

public:
    // 轻量定时器封装：
    // - 以指定 TIM 实例为基础，根据 APB 时钟域计算 PSC/ARR；
    // - 提供用户回调注册与中断启动；
    // - 默认频率 100Hz，可通过构造参数覆盖。
    explicit Timer(TIM_HandleTypeDef *_htim, uint32_t _freqHz = 100);

    // 注册用户回调（在中断上下文调用），仅支持 TIM7/10/11/13/14。
    void SetCallback(TimerCallback_t _timerCallback);

    // 启动定时器并开启更新中断。
    void Start();
};

#endif //REF_STM32F4_TIMER_HPP
