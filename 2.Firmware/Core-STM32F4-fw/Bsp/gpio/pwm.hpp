#ifndef REF_STM32F4_PWM_H
#define REF_STM32F4_PWM_H
/*
 * 基础特性概览（PWM 输出）
 * - 职责：封装 TIM9/TIM12 的四路 PWM 输出，用于驱动舵机/电机等。
 * - 频率：构造时分别设置 A/B 两组频率；占空比范围 0~1。
 * - 通道：CH_A1/CH_A2/CH_B1/CH_B2，支持 CH_ALL 一键设置。
 * - 时序：在控制任务中更新占空比，确保与 10ms 控制周期同步。
 */

#include <cstdint>
#include <tim.h>

class PWM
{
private:

public:
    enum PwmChannel_t
    {
        CH_ALL,
        // TIM
        CH_A1,
        CH_A2,
        CH_B1,
        CH_B2
    };

    // 构造双路(2x2 通道) PWM：TIM9(A1/A2) 与 TIM12(B1/B2)，可分别设定频率。
    explicit PWM(uint32_t _freqHzA = 21000, uint32_t _freqHzB = 21000);

    // 启动指定 PWM 通道（或全部）。初始占空比为 0。
    void Start(PwmChannel_t _channel = CH_ALL);

    // 设置指定通道占空比（0~1），支持一次设置全部通道。
    void SetDuty(PwmChannel_t _channel = CH_ALL, float _duty = 0);

};

#endif //REF_STM32F4_PWM_H
