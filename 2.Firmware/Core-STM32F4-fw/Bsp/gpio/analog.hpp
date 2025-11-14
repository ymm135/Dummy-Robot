#ifndef REF_STM32F4_ANALOG_H
#define REF_STM32F4_ANALOG_H

#include <cstdint>
#include <adc.h>

class Analog
{
private:

public:
    enum AdcChannel_t
    {
        CH1,
        CH2,
        CH3,
        CH4
    };

    Analog()
    = default;

    // 读取芯片内部温度（单位：摄氏度）。
    float GetChipTemperature();

    // 读取指定模拟通道电压（单位：伏）。
    float GetVoltage(AdcChannel_t _channel);

    // 读取指定通道 ADC 原始值（未转换）。
    uint16_t GetRaw(AdcChannel_t _channel);
};

#endif //REF_STM32F4_ANALOG_H
