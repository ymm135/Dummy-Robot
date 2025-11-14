#include "communication.hpp"
#include "dummy_robot.h"
#include <stdio.h>
/*
文件概述：
- DummyRobot 负责关节层调度、模式管理与命令解析，通过 CAN 与驱动交互。
- 主要流程：接收主机命令 -> 解析为 MoveJ/MoveL -> 分配速度/加速度 -> 定时回调采样角度与完成位 -> 反馈当前位姿。
- 单位约定：关节角度使用度(°)；位姿位置输入/输出对外均为毫米(mm)，FK/IK 内部使用米(m)。
- 稳定/顺滑：软限位检查、最大角速度统一总时长、按减速比做角度-电机圈换算、可中断/阻塞模式控制。
*/

inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index) // 返回绝对值最大的关节角及其索引
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (abs(_joints.a[i]) > max)
        {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


// 机器人实例：
// - 初始化每个关节的执行器参数：是否反向、减速比(电机圈/关节圈)、关节角度软限位；
// - 初始化 6 轴运动学求解器的几何参数(单位：米)。
DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, true, 50, -170, 170);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 30, -73, 90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true, 30, 35, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, false, 24, -180, 180);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 30, -120, 120);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true, 50, -720, 720);
    hand = new DummyHand(_hcan, 7);

    dof6Solver = new DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);
}


DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
}


void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
}


void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // 等待所有关节处理完成(驱动侧延迟/应答)
    HAL_NVIC_SystemReset();
}


void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]); // 目标角与初始化偏置之差，按动态速度限下发
    }
}


bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    // 关节空间点到点运动：
    // 1) 边界检查；
    // 2) 计算各关节角度增量(delta)；
    // 3) 用最大角度估算总时长(结合减速比，近似以最大电机圈数/关节速度得到时间)，并分配各关节速度；
    // 4) 设置目标并重置完成标志，随后由回调驱动下发到各关节。
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);
        // 估算总时间：最大关节角度 * 减速比(电机圈/关节圈) / 关节速度(°/s)
        float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        for (int j = 1; j <= 6; j++)
        {
            dynamicJointSpeeds.a[j - 1] =
                abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f); // 统一总时长分配各轴电机速度(0~10 r/s)
        }

        jointsStateFlag = 0; // 清除完成标志，进入运动中
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}


bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    // 直线运动：
    // 1) 调用 IK 求解得到 8 套候选关节解；
    // 2) 逐一检查关节软限位，筛出有效解；
    // 3) 选择与当前姿态差异(最大关节角度)最小的解，保证运动连续；
    // 4) 转交给 MoveJ 以统一边界与速度分配逻辑。
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint); // 选择与当前关节差异最小的解，保证连续性
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

void DummyRobot::UpdateJointAngles()
{
    motorJ[ALL]->UpdateAngle();
}


void DummyRobot::UpdateJointAnglesCallback()
{
    // 由定时回调驱动：采样各关节角度并维护完成状态位（1 表示该关节到达目标）。
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i); // bit1..bit6 为各关节完成位
        else
            jointsStateFlag &= ~(1 << i); // 未完成则清除对应位
    }
}


void DummyRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)_speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio; // 模式相关速度比例系数(轨迹模式降低)
}


void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)_acc = 0;
    else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]); // 各轴基准加速度按百分比缩放
}


void DummyRobot::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome(); // 将当前编码位置写入为零点(偏置)
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting(); // 先回到休止姿态确保偏置一致
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome(); // 再次写零点以消除累计误差
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}


void DummyRobot::Homing()
{
    // 回零过程：临时降低速度，执行两段到位，直至全部停止。
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}


void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    // FK 输出位置单位为米(m)，此处转换为毫米(mm)用于对外接口一致性
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110; // 全 6 轴到位时为 0b1111110(忽略 bit0)
}


bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_MOTOR_TUNING)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1; // 目标点模式：全速、低加速度
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3; // 连续轨迹：降低速度、提高加速度以跟随轨迹
            break;
        case COMMAND_MOTOR_TUNING:
            break;
    }
}


DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint8_t
_id) :
    nodeID(_id), hcan(_hcan)
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


void DummyHand::SetAngle(float _angle)
{
    if (_angle > 30)_angle = 30;
    if (_angle < 0)_angle = 0;

    uint8_t mode = 0x02;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_angle;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetMaxCurrent(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    uint8_t mode = 0x01;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetEnable(bool _enable)
{
    if (_enable)
        SetMaxCurrent(maxCurrent);
    else
        SetMaxCurrent(0);
}


uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo); // 返回剩余空间以便上位机掌握队列负载

    return 0xFF; // failed
}


void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    context->MoveJoints(context->targetJoints);
    context->isEnabled = false; // 禁用周期下发，停止后续命令
    ClearFifo();
}


std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);

    return std::string{strBuffer};
}


uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;
    // 命令语法：'>' 关节角，'@' 位姿；末尾可选速度参数。根据模式选择阻塞/可中断。

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_CONTINUES_TRAJECTORY:
            if (_cmd[0] == '>')
            {
                float joints[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                context->MoveJoints(context->targetJoints);

                while (context->IsMoving() && context->IsEnabled())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                context->MoveJoints(context->targetJoints);

                while (context->IsMoving())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }

            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>')
            {
                float joints[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;
    }

    return osMessageQueueGetSpace(commandFifo);
}


void DummyRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}


void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis) // 正弦扫频：delta = amp * sin(2π f t)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}
