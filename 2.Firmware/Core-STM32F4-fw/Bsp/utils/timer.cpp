#include "timer.hpp"

static TimerCallback_t timerCallbacks[5];

Timer::Timer(TIM_HandleTypeDef *_htim, uint32_t _freqHz)
{
    // 支持的用户定时器实例：TIM7/TIM10/TIM11/TIM13/TIM14
    // 将对应的 HAL 句柄实例化，便于后续校验与分发。
    htim7.Instance = TIM7;
    htim10.Instance = TIM10;
    htim11.Instance = TIM11;
    htim13.Instance = TIM13;
    htim14.Instance = TIM14;

    if (!(_htim->Instance == TIM7 ||
          _htim->Instance == TIM10 ||
          _htim->Instance == TIM11 ||
          _htim->Instance == TIM13 ||
          _htim->Instance == TIM14))
    {
        Error_Handler();
    }

    // 将目标频率限制在安全范围，避免除零或 ARR/PSC 溢出。
    if (_freqHz < 1) _freqHz = 1;
    else if (_freqHz > 10000000) _freqHz = 10000000;

    htim = _htim;
    freq = _freqHz;

    // 根据 APB 时钟域计算 PSC/ARR，使得定时器在目标频率下溢出。
    CalcRegister(freq);

    // 重新初始化底层寄存器：使用计算得到的 PSC/ARR
    HAL_TIM_Base_DeInit(_htim);
    _htim->Init.Prescaler = PSC - 1;
    _htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    _htim->Init.Period = ARR - 1;
    _htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    _htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(_htim) != HAL_OK)
    {
        Error_Handler();
    }
}

void Timer::Start()
{
    HAL_TIM_Base_Start_IT(htim);
}

void Timer::CalcRegister(uint32_t _freq)
{
    float psc = 0.5;
    float arr;

    do
    {
        psc *= 2;
        arr = 84000000.0f / psc / (float) _freq;
    } while (arr > 65535);

    // APB1 域：TIM7/13/14，时钟 84MHz
    if (htim->Instance == TIM7 || htim->Instance == TIM13 || htim->Instance == TIM14) // APB1 @84MHz
    {
        PSC = (uint16_t) round((double) psc);
        ARR = (uint16_t) (84000000.0f / (float) _freq / psc);
    }
    // APB2 域：TIM10/11，定时器时钟为 APB2*2（168MHz*2），此处通过加倍 PSC 抵消。
    else if (htim->Instance == TIM10 || htim->Instance == TIM11) // APB2 @168MHz
    {
        PSC = (uint16_t) round((double) psc) * 2;
        ARR = (uint16_t) (84000000.0f / (float) _freq / psc);
    }
}


void Timer::SetCallback(TimerCallback_t _timerCallback)
{
    // 将用户回调按实例映射到静态表，供中断上下文快速分发。
    if (htim->Instance == TIM7)
    {
        timerCallbacks[0] = _timerCallback;
    } else if (htim->Instance == TIM10)
    {
        timerCallbacks[1] = _timerCallback;
    } else if (htim->Instance == TIM11)
    {
        timerCallbacks[2] = _timerCallback;
    } else if (htim->Instance == TIM13)
    {
        timerCallbacks[3] = _timerCallback;
    } else if (htim->Instance == TIM14)
    {
        timerCallbacks[4] = _timerCallback;
    }
}


extern "C"
void OnTimerCallback(TIM_TypeDef *timInstance)
{
    // 从 HAL 层转发进来的定时器更新中断，依据具体实例调用对应用户回调。
    if (timInstance == TIM7)
    {
        timerCallbacks[0]();
    } else if (timInstance == TIM10)
    {
        timerCallbacks[1]();
    } else if (timInstance == TIM11)
    {
        timerCallbacks[2]();
    } else if (timInstance == TIM13)
    {
        timerCallbacks[3]();
    } else if (timInstance == TIM14)
    {
        timerCallbacks[4]();
    }
}