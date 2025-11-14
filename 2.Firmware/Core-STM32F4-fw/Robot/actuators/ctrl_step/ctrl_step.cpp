#include "ctrl_step.hpp"
#include "communication.hpp"


// CAN 报文说明：
// - 使用标准帧 StdId，格式为 (nodeID << 7) | mode；
// - 数据区按驱动协议约定，浮点/整数通过字节序列发送；
// - 位置/速度/电流等命令的单位以驱动固件为准，此文件负责关节角与电机圈的换算。
// - 模式码约定：0x01 使能，0x02 校准，0x03 电流设定，0x04 速度设定，0x05 位置设定，0x07 位置+速度限，
//   0x11~0x1B 参数写入，0x23 角度查询，0x7E(擦除配置)/0x7F(重启)。
// - 字节序：浮点/整型均按 little-endian 发送；canBuf[4] 作为“需要保存/应答”标志，含义随不同命令约定。
// 构造：指定节点 ID、方向反转、减速比(电机圈/关节圈)与关节角度软限位。
CtrlStepMotor::CtrlStepMotor(CAN_HandleTypeDef* _hcan, uint8_t _id, bool _inverse,
                             uint8_t _reduction, float _angleLimitMin, float _angleLimitMax) :
    nodeID(_id), hcan(_hcan), inverseDirection(_inverse), reduction(_reduction),
    angleLimitMin(_angleLimitMin), angleLimitMax(_angleLimitMax)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}


void CtrlStepMotor::SetEnable(bool _enable)
{
    state = _enable ? FINISH : STOP; // 本地状态：使能视为“待命/已到位”，关闭视为停止

    uint8_t mode = 0x01;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::DoCalibration()
{
    uint8_t mode = 0x02; // 启动驱动侧校准/回零流程
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;

    uint8_t mode = 0x03;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocitySetPoint(float _val)
{
    state = RUNNING;

    uint8_t mode = 0x04;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionSetPoint(float _val)
{
    uint8_t mode = 0x05;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need ACK

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionWithVelocityLimit(float _pos, float _vel)
{
    uint8_t mode = 0x07;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_pos;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    b = (unsigned char*) &_vel; // 速度上限(单位由驱动侧定义)
    for (int i = 4; i < 8; i++)
        canBuf[i] = *(b + i - 4);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetNodeID(uint32_t _id)
{
    uint8_t mode = 0x11;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    auto* b = (unsigned char*) &_id;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentLimit(float _val)
{
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocityLimit(float _val)
{
    uint8_t mode = 0x13;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAcceleration(float _val)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 0; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::ApplyPositionAsHome()
{
    uint8_t mode = 0x15;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetEnableOnBoot(bool _enable)
{
    uint8_t mode = 0x16;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetEnableStallProtect(bool _enable)
{
    uint8_t mode = 0x1B;
    txHeader.StdId = nodeID << 7 | mode;

    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::Reboot()
{
    uint8_t mode = 0x7f;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::EraseConfigs()
{
    uint8_t mode = 0x7e;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


// 设置目标关节角度：按约定将关节角(°)换算成电机圈并下发到位置控制。
void CtrlStepMotor::SetAngle(float _angle)
{
    _angle = inverseDirection ? -_angle : _angle;
    // 角度 -> 电机圈：joint_deg / 360 * reduction
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionSetPoint(stepMotorCnt);
}


// 设置目标角度并限制速度：同样以减速比做角度换算。
void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithVelocityLimit(stepMotorCnt, _vel);
}


void CtrlStepMotor::UpdateAngle()
{
    uint8_t mode = 0x23; // 角度查询命令
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader); // 驱动侧完成后会通过对应回调返回角度
}


// 回读角度回调：将电机圈换算回关节角度(°)，并维护运行状态。
void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;

    // 电机圈 -> 角度：motor_turns / reduction * 360°
    float tmp = _pos / (float) reduction * 360;
    angle = inverseDirection ? -tmp : tmp; // 统一到关节角(°)，考虑方向反转
}


// 参数调试：位置/速度/电流闭环系数写入 (DCE Kp/Kv/Ki/Kd)，具体含义由驱动固件定义
void CtrlStepMotor::SetDceKp(int32_t _val)
{
    uint8_t mode = 0x17;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKv(int32_t _val)
{
    uint8_t mode = 0x18;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKi(int32_t _val)
{
    uint8_t mode = 0x19;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKd(int32_t _val)
{
    uint8_t mode = 0x1A;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}
