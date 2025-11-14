#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include <string>

#define ALL 0

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 2               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 2               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */


class DummyHand
{
public:
    uint8_t nodeID = 7;
    float maxCurrent = 0.7;


    DummyHand(CAN_HandleTypeDef* _hcan, uint8_t _id);


    void SetAngle(float _angle);
    void SetMaxCurrent(float _val);
    void SetEnable(bool _enable);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("set_angle", *this, &DummyHand::SetAngle, "angle"),
            make_protocol_function("set_enable", *this, &DummyHand::SetEnable, "enable"),
            make_protocol_function("set_current_limit", *this, &DummyHand::SetMaxCurrent, "current")
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8];
    CAN_TxHeaderTypeDef txHeader;
    float minAngle = 0;
    float maxAngle = 45;
};


/**
 * @brief DummyRobot：机械臂高层控制封装
 * - 维护当前/目标关节与末端位姿；
 * - 提供 MoveJ/MoveL 规划接口(调用 IK/FK 与边界检查)；
 * - 管理命令模式、速度/加速度与调试辅助；
 * - 将关节角度下发到执行器(CtrlStepMotor)，并从中回读状态。
 */
class DummyRobot
{
public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();


    // 命令模式：
    // - SEQUENTIAL：顺序到达目标点，队列执行；
    // - INTERRUPTABLE：可被新目标打断；
    // - CONTINUES_TRAJECTORY：连续轨迹；
    // - MOTOR_TUNING：电机调试模式（频率/幅值）。
    /**
     * @brief 命令模式
     * - SEQUENTIAL：顺序到达目标点，队列执行；
     * - INTERRUPTABLE：可被新目标打断；
     * - CONTINUES_TRAJECTORY：连续轨迹；
     * - MOTOR_TUNING：电机调试模式（频率/幅值）。
     */
    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context)
        {
        }

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);


        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }


    private:
        DummyRobot* context;
        float time = 0;
        uint8_t tuningFlag = 0;
        float frequency = 1;
        float amplitude = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);


    // This is the pose when power on.
    /** 上电默认姿态(°) */
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -73, 180, 0, 0, 0};
    /** 默认关节速度(°/s) */
    const float DEFAULT_JOINT_SPEED = 30;
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
    /** 加速度档位下限(0~100) */
    const float DEFAULT_JOINT_ACCELERATION_LOW = 30;
    /** 加速度档位上限(0~100) */
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;


    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor* motorJ[7] = {nullptr};
    DummyHand* hand = {nullptr};


    /** @brief 初始化机器人：设置初始模式/速度等 */
    void Init();
    /** @brief MoveJ：以关节空间点到点运动到指定姿态；返回 false 表示触及软限或无效 */
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    /** @brief MoveL：以末端直线到达指定位姿（含 IK 求解与解连续性选择）；返回 false 表示无有效解 */
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    /** @brief 直接下发关节角度（带速度限制） */
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    /** @brief 设置关节速度(°/s)，范围 0~100，经内部比例 jointSpeedRatio 作用 */
    void SetJointSpeed(float _speed);
    /** @brief 设置加速度档位(0~100)，按每轴基准加速度缩放 */
    void SetJointAcceleration(float _acc);
    /** @brief 采样并更新关节角度与完成标志 */
    void UpdateJointAngles();
    /** @brief 定时回调：维护当前角度与完成状态位 */
    void UpdateJointAnglesCallback();
    /** @brief 更新末端位姿（FK，位置单位转换为毫米用于上位机一致性） */
    void UpdateJointPose6D();
    /** @brief 重启所有节点/MCU */
    void Reboot();
    /** @brief 总使能开关（透传到各关节） */
    void SetEnable(bool _enable);
    /** @brief Home 偏移标定流程（两次置零与复位） */
    void CalibrateHomeOffset();
    /** @brief 回零流程：暂降速度，两段到位 */
    void Homing();
    /** @brief 休息位姿：以低速到达 REST_POSE */
    void Resting();
    /** @brief 是否仍在运动（基于完成状态位） */
    bool IsMoving();
    /** @brief 是否已使能 */
    bool IsEnabled();
    /** @brief 设置命令模式（见 CommandMode） */
    void SetCommandMode(uint32_t _mode);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DummyRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DummyRobot::Homing),
            make_protocol_function("resting", *this, &DummyRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot", *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable", *this, &DummyRobot::SetEnable, "enable"),
            make_protocol_function("move_j", *this, &DummyRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6"),
            make_protocol_function("move_l", *this, &DummyRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("set_joint_speed", *this, &DummyRobot::SetJointSpeed, "speed"),
            make_protocol_function("set_joint_acc", *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode, "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions())
        );
    }


    /**
     * @brief 命令处理器：解析 ASCII 命令并入队/出队
     */
    class CommandHandler
    {
    public:
        explicit CommandHandler(DummyRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }

        /** @brief 入队命令字符串，返回剩余空间 */
        uint32_t Push(const std::string &_cmd);
        /** @brief 出队命令，支持阻塞等待 timeout(ms) */
        std::string Pop(uint32_t timeout);
        /** @brief 解析并执行命令（根据 CommandMode 分流），返回剩余空间 */
        uint32_t ParseCommand(const std::string &_cmd);
        /** @brief 获取队列剩余空间 */
        uint32_t GetSpace();
        /** @brief 清空命令队列 */
        void ClearFifo();
        /** @brief 急停（预留） */
        void EmergencyStop();


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
        char strBuffer[64]{};
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic* dof6Solver;
    bool isEnabled = false;
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
