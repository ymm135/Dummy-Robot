#ifndef REF_STM32F4_ANALOG_H
#define REF_STM32F4_ANALOG_H
/*
 * 基础特性概览（模拟量采样）
 * - 职责：封装 ADC 通道读取与芯片内部温度计算。
 * - 通道映射：CH1~CH4 对应板载模拟输入；通过 HAL ADC 获取原始值。
 * - 单位：GetVoltage 返回伏特；GetRaw 返回原始码；温度为摄氏度。
 * - 时序：建议在采样任务中调用，避免 ISR 做浮点换算。
 */

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
