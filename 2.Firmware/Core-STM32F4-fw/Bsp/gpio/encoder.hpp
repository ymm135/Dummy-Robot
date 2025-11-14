#ifndef REF_STM32F4_ENCODER_HPP
#define REF_STM32F4_ENCODER_HPP
/*
 * 基础特性概览（编码器接口）
 * - 职责：封装 TIM 编码器模式，提供累计计数与角度计算（度/弧度）。
 * - 数据来源：TIM2/TIM3 16bit 计数配合 encCntLoop 实现 64bit 累加，避免溢出。
 * - 配置项：cpr（每转脉冲数），inverse（方向反转）。
 * - 协议：通过 MakeProtocolDefinitions 暴露属性与函数至 Fibre。
 * - 使用：构造传入 HAL TIM 句柄；Start 启动；GetCount/GetAngle 查询。
 */

#include <fibre/protocol.hpp>
#include "tim.h"

class Encoder
{
private:
    const float RAD_TO_DEG = 57.295777754771045f;

    TIM_HandleTypeDef *htim;

public:
    explicit Encoder(TIM_HandleTypeDef *_htim, uint16_t _cpr = 4096, bool _inverse = false);

    void Start();

    // 读取编码器累计计数（含溢出处理），用于角度计算。
    int64_t GetCount();

    // 获取当前角度；默认单位为度，可通过 _useRAD=true 切换为弧度。
    float GetAngle(bool _useRAD = false);

    struct Config_t
    {
        uint16_t cpr;

        bool inverse;
    };

    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_object("config",
                                 make_protocol_property("cpr", &config.cpr),
                                 make_protocol_property("inverse", &config.inverse)
            ),
            make_protocol_function("get_count", *this, &Encoder::GetCount),
            make_protocol_function("get_angle", *this, &Encoder::GetAngle, "use_rad")
        );
    }

    Config_t config;
};

#endif //REF_STM32F4_ENCODER_HPP
