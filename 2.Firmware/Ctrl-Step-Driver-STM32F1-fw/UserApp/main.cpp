#include "common_inc.h"
#include "configurations.h"
#include "Platform/Utils/st_hardware.h"
#include <tim.h>


/* Component Definitions -----------------------------------------------------*/
BoardConfig_t boardConfig;
Motor motor;
TB67H450 tb67H450;
MT6816 mt6816;
EncoderCalibrator encoderCalibrator(&motor);
Button button1(1, 1000), button2(2, 3000);
void OnButton1Event(Button::Event _event);
void OnButton2Event(Button::Event _event);
Led statusLed;


/* Main Entry ----------------------------------------------------------------*/
// 系统主入口：
// - 根据唯一序列号映射默认关节 NodeID（J1~J6）；
// - 应用 EEPROM 中的电机与控制参数（若无则写入默认值）；
// - 绑定驱动/编码器并初始化；
// - 启动 100Hz/20kHz 两个定时器中断，分别用于慢速任务与闭环控制；
// - 按键事件：Button1 切换/停止模式、Button2 清除堵转或快速归零。
void Main()
{
    uint64_t serialNum = GetSerialNumber();
    uint16_t defaultNodeID = 0;
    // Change below to fit your situation
    switch (serialNum)
    {
        case 431466563640: //J1
            defaultNodeID = 1;
            break;
        case 384624576568: //J2
            defaultNodeID = 2;
            break;
        case 384290670648: //J3
            defaultNodeID = 3;
            break;
        case 431531051064: //J4
            defaultNodeID = 4;
            break;
        case 431466760248: //J5
            defaultNodeID = 5;
            break;
        case 431484848184: //J6
            defaultNodeID = 6;
            break;
        default:
            break;
    }


    /*---------- Apply EEPROM Settings ----------*/
    // 设置优先级：EEPROM > Motor.h 默认参数
    // 字段含义：
    // - currentLimit：额定电流(A)
    // - velocityLimit：额定速度(电机细分步/秒)，等效“r/s * 细分步/圈”
    // - velocityAcc：额定加速度(细分步/秒^2)
    // - dce_*：DCE 控制器参数(kp/kv/ki/kd)
    // - enableMotorOnBoot/enableStallProtect：上电使能与堵转保护开关
    // - encoderHomeOffset：编码器零点偏移(细分步)
    EEPROM eeprom;
    eeprom.get(0, boardConfig);
    if (boardConfig.configStatus != CONFIG_OK) // use default settings
    {
        boardConfig = BoardConfig_t{
            .configStatus = CONFIG_OK,
            .canNodeId = defaultNodeID,
            .encoderHomeOffset = 0,
            .defaultMode = Motor::MODE_COMMAND_POSITION,
            .currentLimit = 1 * 1000,    // A
            .velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s
            .velocityAcc = 100 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2
            .calibrationCurrent=2000,
            .dce_kp = 200,
            .dce_kv = 80,
            .dce_ki = 300,
            .dce_kd = 250,
            .enableMotorOnBoot=false,
            .enableStallProtect=false
        };
        eeprom.put(0, boardConfig);
    }
    motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
    motor.config.motionParams.ratedCurrent = boardConfig.currentLimit;
    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
    motor.config.motionParams.ratedVelocityAcc = boardConfig.velocityAcc;
    // 规划器加速度设置：统一使用 EEPROM 配置
    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.config.motionParams.caliCurrent = boardConfig.calibrationCurrent;
    motor.config.ctrlParams.dce.kp = boardConfig.dce_kp;
    motor.config.ctrlParams.dce.kv = boardConfig.dce_kv;
    motor.config.ctrlParams.dce.ki = boardConfig.dce_ki;
    motor.config.ctrlParams.dce.kd = boardConfig.dce_kd;
    motor.config.ctrlParams.stallProtectSwitch = boardConfig.enableStallProtect;


    /*---------------- Init Motor ----------------*/
    motor.AttachDriver(&tb67H450);
    motor.AttachEncoder(&mt6816);
    motor.controller->Init();
    motor.driver->Init();
    motor.encoder->Init();


    /*------------- Init peripherals -------------*/
    button1.SetOnEventListener(OnButton1Event);
    button2.SetOnEventListener(OnButton2Event);


    /*------- Start Close-Loop Control Tick ------*/
    HAL_Delay(100);
    // 定时器：
    // - TIM1: 100Hz 慢速任务(按钮扫描、状态灯、通信处理等)
    // - TIM4: 20kHz 快速闭环控制(编码器采样、PWM/步进驱动输出)
    HAL_TIM_Base_Start_IT(&htim1);  // 100Hz
    HAL_TIM_Base_Start_IT(&htim4);  // 20kHz

    if (button1.IsPressed() && button2.IsPressed())
        encoderCalibrator.isTriggered = true;


    for (;;)
    {
        encoderCalibrator.TickMainLoop();


        if (boardConfig.configStatus == CONFIG_COMMIT)
        {
            boardConfig.configStatus = CONFIG_OK;
            eeprom.put(0, boardConfig);
        } else if (boardConfig.configStatus == CONFIG_RESTORE)
        {
            eeprom.put(0, boardConfig);
            HAL_NVIC_SystemReset();
        }
    }
}


/* Event Callbacks -----------------------------------------------------------*/
extern "C" void Tim1Callback100Hz()
{
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    // 10ms Tick：按钮扫描与状态灯显示随控制器状态变化
    button1.Tick(10);
    button2.Tick(10);
    statusLed.Tick(10, motor.controller->state);
}


extern "C" void Tim4Callback20kHz()
{
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

    // 20kHz Tick：优先执行编码器标定，否则执行正常电机闭环
    if (encoderCalibrator.isTriggered)
        encoderCalibrator.Tick20kHz();
    else
        motor.Tick20kHz();
}


// Button1：单击在默认模式与 STOP 间切换；长按触发系统复位
void OnButton1Event(Button::Event _event)
{
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            HAL_NVIC_SystemReset();
            break;
        case ButtonBase::CLICK:
            // 若当前运行非 STOP，则保存当前模式为默认并请求 STOP
            if (motor.controller->modeRunning != Motor::MODE_STOP)
            {
                boardConfig.defaultMode = motor.controller->modeRunning;
                motor.controller->requestMode = Motor::MODE_STOP;
            } else
            {
                // 否则按 EEPROM 记录的默认模式恢复运行
                motor.controller->requestMode = static_cast<Motor::Mode_t>(boardConfig.defaultMode);
            }
            break;
    }
}


// Button2：长按快速清零当前模式的目标；单击清除堵转标志
void OnButton2Event(Button::Event _event)
{
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            switch (motor.controller->modeRunning)
            {
                case Motor::MODE_COMMAND_CURRENT:
                case Motor::MODE_PWM_CURRENT:
                    motor.controller->SetCurrentSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_VELOCITY:
                case Motor::MODE_PWM_VELOCITY:
                    motor.controller->SetVelocitySetPoint(0);
                    break;
                case Motor::MODE_COMMAND_POSITION:
                case Motor::MODE_PWM_POSITION:
                    motor.controller->SetPositionSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_Trajectory:
                case Motor::MODE_STEP_DIR:
                case Motor::MODE_STOP:
                    break;
            }
            break;
        case ButtonBase::CLICK:
            // 清除堵转保护标志，允许重新运行
            motor.controller->ClearStallFlag();
            break;
    }
}