#ifndef DUMMY_CORE_FW_CTRL_STEP_HPP
#define DUMMY_CORE_FW_CTRL_STEP_HPP

#include "fibre/protocol.hpp"
#include "can.h"

/**
 * CtrlStepMotor
 * @brief 步进电机控制器适配层：在“关节角(°)”与电机侧“圈数”之间做线性换算，
 *        并通过 CAN 将位置/速度/电流等目标下发给电机驱动板。
 *
 * 关键约定与单位：
 * - reduction：减速比，定义为“电机圈/关节圈”，即关节旋转 360° 所需电机旋转的圈数；
 * - angle：以度(°)表示的关节角；
 * - SetAngle/UpdateAngle：在角度与电机圈之间使用 reduction 做线性换算；
 * - 其它 SetXXXLimit/SetXXXSetPoint 的数值单位与驱动板协议一致，见各方法说明。
 */

class CtrlStepMotor
{
public:
    // 步进电机控制器：负责将关节角度/速度等指令转换为电机侧“圈数/速度”并通过 CAN 下发。
    // 关键约定：
    // - reduction：减速比，定义为“电机圈/关节圈”，即关节转 360° 需要电机转多少圈；
    // - angle 单位：度(°)；
    // - SetAngle/UpdateAngle：在角度与电机圈之间使用 reduction 做线性换算。
    enum State
    {
        RUNNING,
        FINISH,
        STOP
    };


    // 控制器内部每圈脉冲计数(仅供参考)：200 步/圈，256 细分 -> 51200 脉冲/电机圈
    const uint32_t CTRL_CIRCLE_COUNT = 200 * 256;

    /**
     * @brief 构造函数
     * @param _hcan       CAN 句柄
     * @param _id         节点 ID（驱动板逻辑节点号）
     * @param _inverse    是否反向（true 时下发/回读角度取负）
     * @param _reduction  减速比：电机圈/关节圈
     * @param _angleLimitMin  关节角软限下限(°)
     * @param _angleLimitMax  关节角软限上限(°)
     */
    CtrlStepMotor(CAN_HandleTypeDef* _hcan, uint8_t _id, bool _inverse = false, uint8_t _reduction = 1,
                  float _angleLimitMin = -180, float _angleLimitMax = 180);

    uint8_t nodeID;
    float angle = 0;           // 当前关节角度(°)，由回读位置换算得到
    float angleLimitMax;
    float angleLimitMin;
    bool inverseDirection;     // 方向反转标志：true 则角度取负号
    uint8_t reduction;         // 减速比：电机圈/关节圈，用于角度与电机位置换算
    State state = STOP;

    /** @brief 设置目标关节角(°)，内部换算为电机圈并下发到位置控制 */
    void SetAngle(float _angle);
    /** @brief 设置目标关节角(°)，并限制速度(°/s)，内部按减速比换算 */
    void SetAngleWithVelocityLimit(float _angle, float _vel);
    // CAN Command
    /** @brief 使能/关闭驱动（true=使能，false=关闭） */
    void SetEnable(bool _enable);
    /** @brief 执行驱动侧校准流程（原点/编码器对齐等） */
    void DoCalibration();
    /** @brief 设置电流设定值(A) */
    void SetCurrentSetPoint(float _val);
    /** @brief 设置速度设定值(圈/s 或驱动内部单位) */
    void SetVelocitySetPoint(float _val);
    /** @brief 设置位置设定值(电机圈) */
    void SetPositionSetPoint(float _val);
    /** @brief 设置位置(电机圈)并通过时间/速度限制进行整形 */
    void SetPositionWithVelocityLimit(float _pos, float _vel);
    /** @brief 设置节点 ID（并可选择是否固化） */
    void SetNodeID(uint32_t _id);
    /** @brief 设置电流上限(A) */
    void SetCurrentLimit(float _val);
    /** @brief 设置速度上限(圈/s) */
    void SetVelocityLimit(float _val);
    /** @brief 设置加速度参数（驱动内部单位） */
    void SetAcceleration(float _val);
    /** @brief DCE 控制参数 Kp */
    void SetDceKp(int32_t _val);
    /** @brief DCE 控制参数 Kv */
    void SetDceKv(int32_t _val);
    /** @brief DCE 控制参数 Ki */
    void SetDceKi(int32_t _val);
    /** @brief DCE 控制参数 Kd */
    void SetDceKd(int32_t _val);
    /** @brief 将当前位置应用为 Home 偏移（相当于置零） */
    void ApplyPositionAsHome();
    /** @brief 设置上电自动使能 */
    void SetEnableOnBoot(bool _enable);
    /** @brief 使能堵转保护 */
    void SetEnableStallProtect(bool _enable);
    /** @brief 重启驱动板 */
    void Reboot();
    /** @brief 擦除保存的配置 */
    void EraseConfigs();

    /** @brief 请求驱动回传当前位置（电机圈） */
    void UpdateAngle();
    /** @brief 角度回读回调：电机圈->关节角(°)，并更新运行状态 */
    void UpdateAngleCallback(float _pos, bool _isFinished);


    // Communication protocol definitions
    /** @brief 发布到 Fibre 的协议端点定义 */
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_ro_property("angle", &angle),
            make_protocol_function("reboot", *this, &CtrlStepMotor::Reboot),
            make_protocol_function("erase_configs", *this, &CtrlStepMotor::EraseConfigs),
            make_protocol_function("set_enable", *this, &CtrlStepMotor::SetEnable, "enable"),
            make_protocol_function("set_position_with_time", *this,
                                   &CtrlStepMotor::SetPositionWithVelocityLimit, "pos", "time"),
            make_protocol_function("set_position", *this, &CtrlStepMotor::SetPositionSetPoint, "pos"),
            make_protocol_function("set_velocity", *this, &CtrlStepMotor::SetVelocitySetPoint, "vel"),
            make_protocol_function("set_velocity_limit", *this, &CtrlStepMotor::SetVelocityLimit, "vel"),
            make_protocol_function("set_current", *this, &CtrlStepMotor::SetCurrentSetPoint, "current"),
            make_protocol_function("set_current_limit", *this, &CtrlStepMotor::SetCurrentLimit, "current"),
            make_protocol_function("set_node_id", *this, &CtrlStepMotor::SetNodeID, "id"),
            make_protocol_function("set_acceleration", *this, &CtrlStepMotor::SetAcceleration, "acc"),
            make_protocol_function("apply_home_offset", *this, &CtrlStepMotor::ApplyPositionAsHome),
            make_protocol_function("do_calibration", *this, &CtrlStepMotor::DoCalibration),
            make_protocol_function("set_enable_on_boot", *this, &CtrlStepMotor::SetEnableOnBoot, "enable"),
            make_protocol_function("set_dce_kp", *this, &CtrlStepMotor::SetDceKp, "vel"),
            make_protocol_function("set_dce_kv", *this, &CtrlStepMotor::SetDceKv, "vel"),
            make_protocol_function("set_dce_ki", *this, &CtrlStepMotor::SetDceKi, "vel"),
            make_protocol_function("set_dce_kd", *this, &CtrlStepMotor::SetDceKd, "vel"),
            make_protocol_function("set_enable_stall_protect", *this, &CtrlStepMotor::SetEnableStallProtect,
                                   "enable"),
            make_protocol_function("update_angle", *this, &CtrlStepMotor::UpdateAngle)
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = {};
};

#endif //DUMMY_CORE_FW_CTRL_STEP_HPP
