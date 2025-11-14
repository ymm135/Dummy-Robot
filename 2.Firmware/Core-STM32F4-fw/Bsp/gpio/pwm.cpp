#include "pwm.hpp"

PWM::PWM(uint32_t _freqHzA, uint32_t _freqHzB)
{
    // 频率保护：限制在 10~84kHz，有效避免分频系数异常。
    if (_freqHzA > 84000)_freqHzA = 84000;
    else if (_freqHzA < 10)_freqHzA = 10;

    if (_freqHzB > 84000)_freqHzB = 84000;
    else if (_freqHzB < 10)_freqHzB = 10;


    TIM_OC_InitTypeDef sConfigOC = {0};

    htim9.Instance = TIM9;
    // TIM9(APB2): 168MHz/分频 -> 计数频率，Period=999 实现 1000 份占空比。
    htim9.Init.Prescaler = 168000 / _freqHzA - 1;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = 999;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim9);

    htim12.Instance = TIM12;
    // TIM12(APB1): 84MHz/分频 -> 计数频率，Period=999 同上。
    htim12.Init.Prescaler = 84000 / _freqHzB - 1;
    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim12.Init.Period = 999;
    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim12);

}

void PWM::Start(PWM::PwmChannel_t _channel)
{
    // 启动指定通道 PWM 输出；默认将 CCR 设为 0（占空比 0）。
    switch (_channel)
    {
        case CH_A1:
            TIM9->CCR1 = 0;
            HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
            break;
        case CH_A2:
            TIM9->CCR2 = 0;
            HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
            break;
        case CH_B1:
            TIM12->CCR1 = 0;
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
            break;
        case CH_B2:
            TIM12->CCR2 = 0;
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
            break;
        case CH_ALL:
            TIM9->CCR1 = 0;
            HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
            TIM9->CCR2 = 0;
            HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
            TIM12->CCR1 = 0;
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
            TIM12->CCR2 = 0;
            HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
            break;
    }
}

void PWM::SetDuty(PWM::PwmChannel_t _channel, float _duty)
{
    if (_duty > 1)_duty = 1;
    else if (_duty < 0)_duty = 0;

    auto ccr = static_cast<uint32_t>(1000 * _duty);

    // 将占空比映射到 CCR（0~999），并应用到对应通道。
    switch (_channel)
    {
        case CH_A1:
            TIM9->CCR1 = ccr;
            break;
        case CH_A2:
            TIM9->CCR2 = ccr;
            break;
        case CH_B1:
            TIM12->CCR1 = ccr;
            break;
        case CH_B2:
            TIM12->CCR2 = ccr;
            break;
        case CH_ALL:
            TIM9->CCR1 = ccr;
            TIM9->CCR2 = ccr;
            TIM12->CCR1 = ccr;
            TIM12->CCR2 = ccr;
            break;
    }
}
