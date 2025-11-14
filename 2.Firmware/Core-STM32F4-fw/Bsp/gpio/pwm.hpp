#ifndef REF_STM32F4_PWM_H
#define REF_STM32F4_PWM_H

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
