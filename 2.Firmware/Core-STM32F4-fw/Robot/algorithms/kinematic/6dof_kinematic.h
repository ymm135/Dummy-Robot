#ifndef DOF6_KINEMATIC_SOLVER_H
#define DOF6_KINEMATIC_SOLVER_H

#include "stm32f405xx.h"
#include "arm_math.h"
#include "memory.h"

// 6 轴机械臂运动学求解器：
// - 使用标准 DH 参数描述连杆与关节关系；
// - 提供正运动学(FK)与逆运动学(IK)求解接口；
// - 外部角度单位为度(°)，姿态欧拉角也使用度；
// - 位姿位置单位：FK 输出为米(m)，IK 输入建议为毫米(mm)，内部会换算为米。

class DOF6Kinematic
{
private:
    const float RAD_TO_DEG = 57.295777754771045f;

    // DH 参数定义：
    // L_BASE: 底座高度(基坐标系到关节1 Z 方向位移)；
    // D_BASE: 底座水平偏移(基坐标系到关节1 X/Y 投影中的 X 分量)；
    // L_ARM: 上臂长度(关节2到关节3 连杆长度)；
    // L_FOREARM: 前臂长度(关节3到关节4 连杆长度)；
    // D_ELBOW: 肘部偏移(肘关节的侧向/竖向偏置，影响几何三角关系)；
    // L_WRIST: 腕部长度(末端到法兰的长度)。
    struct ArmConfig_t
    {
        float L_BASE;
        float D_BASE;
        float L_ARM;
        float L_FOREARM;
        float D_ELBOW;
        float L_WRIST;
    };
    ArmConfig_t armConfig;

    // DH 表：每行为 [theta_offset, d, a, alpha]
    // - theta_offset: 该关节的常量角度偏置(θ0)，用于坐标系对齐；
    // - d: 沿 Z 轴的位移；
    // - a: 沿 X 轴的位移(连杆长度)；
    // - alpha: 相邻坐标系间的扭转角(绕 X 轴)。
    float DH_matrix[6][4] = {0};
    float L1_base[3] = {0};
    float L2_arm[3] = {0};
    float L3_elbow[3] = {0};
    float L6_wrist[3] = {0};

    float l_se_2;
    float l_se;
    float l_ew_2;
    float l_ew;
    float atan_e;

public:
    struct Joint6D_t
    {
        Joint6D_t()
        = default;

        Joint6D_t(float a1, float a2, float a3, float a4, float a5, float a6)
            : a{a1, a2, a3, a4, a5, a6}
        {}

        float a[6]; // 关节角，单位：度(°)，顺序 J1~J6

        friend Joint6D_t operator-(const Joint6D_t &_joints1, const Joint6D_t &_joints2);
    };

    struct Pose6D_t
    {
        Pose6D_t()
        = default;

        Pose6D_t(float x, float y, float z, float a, float b, float c)
            : X(x), Y(y), Z(z), A(a), B(b), C(c), hasR(false)
        {}

        float X{}, Y{}, Z{}; // 位置：通常以毫米(mm)传入 IK；FK 输出为米(m)
        float A{}, B{}, C{}; // 欧拉角(度)：绕 ZYX 或 XYZ 的具体定义见实现
        float R[9]{};        // 旋转矩阵(行优先 3x3)，若手动设置姿态可直接提供

        // if Pose was calculated by FK then it's true automatically (so that no need to do extra calc),
        // otherwise if manually set params then it should be set to false.
        bool hasR{}; // true 表示 R 已就绪，无需由欧拉角再计算
    };

    struct IKSolves_t
    {
        Joint6D_t config[8];
        char solFlag[8][3];
    };

    DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);

    // 正运动学：输入关节角(度)，输出末端位姿(位置 m、姿态度，并给出旋转矩阵)
    bool SolveFK(const Joint6D_t &_inputJoint6D, Pose6D_t &_outputPose6D);

    // 逆运动学：输入末端位姿(位置建议 mm，姿态度或 R)，以及上一时刻关节角(用于选择连续解)
    // 输出 8 组候选解(config[0..7])，并通过 solFlag 标记肩/肘/腕的解存在性与奇异性。
    bool SolveIK(const Pose6D_t &_inputPose6D, const Joint6D_t &_lastJoint6D, IKSolves_t &_outputSolves);
};

#endif //DOF6_KINEMATIC_SOLVER_H
