#include "common_inc.h"
/*
 * 文件总览（UserApp / main.cpp）
 * - 职责：用户应用入口与主控制逻辑，组织通信、机器人、传感器与显示，并启动线程与控制定时器。
 * - 线程：固定周期控制线程（实时）、命令处理线程（普通）、OLED 展示线程（普通）。
 * - 定时器：TIM7 作为控制节拍（当前设置 200Hz），在 ISR 中以通知唤醒控制线程并即时切换。
 * - 数据通道：USB/UART/CAN 通过通信栈统一封装为 Fibre StreamSink；命令采用 ASCII/二进制协议解析。
 * - 关键约束：ISR 轻量（仅通知/唤醒），解析/控制在任务中执行；避免长阻塞与大量日志影响控制一致性。
 */


// 基础资源实例：显示(SSD1306)、IMU(MPU6050)、控制定时器(TIM7@200Hz)、PWM(TIM9/TIM12)、机器人(CAN1)
// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 200);  // 控制节拍定时器：TIM7，200Hz（5ms），用于固定周期关节控制
// 2x2-channel PWMs, used htim9 & htim12, each has 2-channel outputs
PWM pwm(21000, 21000); // PWM 输出：TIM9/TIM12 各两路，默认频率 21kHz
// Robot instance
DummyRobot dummy(&hcan1); // 机器人实例：通过 CAN1 与驱动器通信


/* Thread Definitions -----------------------------------------------------*/
osThreadId_t controlLoopFixUpdateHandle;
void ThreadControlLoopFixUpdate(void* argument)
{
    // 固定周期控制线程：
    // 由 TIM7 中断通过通知唤醒；根据当前命令模式进行关节控制或仅状态更新。
    // 周期内任务划分：
    // - 已使能：根据 commandMode 执行轨迹/点位/调参，下发电机命令；
    // - 未使能：仅采样与姿态计算，不下发命令；
    // - 每周期统一更新 6D 位姿，保证显示与上行一致。
    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (dummy.IsEnabled())
        {
            // Send control command to Motors & update Joint states
            switch (dummy.commandMode)
            {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints);
                    dummy.UpdateJointPose6D();
                    break;
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    dummy.UpdateJointPose6D();
                    break;
            }
        } else
        {
            // Just update Joint states
            dummy.UpdateJointAngles();
            dummy.UpdateJointPose6D();
        }
    }
}


osThreadId_t ControlLoopUpdateHandle;
void ThreadControlLoopUpdate(void* argument)
{
    // 命令处理线程：阻塞等待 USB/串口等上行命令，解析后下发控制。
    // 通道来源：USB-VCP/UART4/UART5/CAN；Pop(osWaitForever) 阻塞从队列取一条待处理命令。
    for (;;)
    {
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
    }
}


osThreadId_t oledTaskHandle;
void ThreadOledUpdate(void* argument)
{
    // OLED 状态展示线程：周期采样 IMU 与机器人状态，绘制到屏幕。
    uint32_t t = micros();
    char buf[16];
    char cmdModeNames[4][4] = {"SEQ", "INT", "TRJ", "TUN"};

    // 显示策略：用 micros() 计算 FPS；绘制 6 关节角与 6D 位姿，并标识电机状态位。

    for (;;)
    {
        mpu6050.Update(true);

        oled.clearBuffer();
        oled.setFont(u8g2_font_5x8_tr);
        oled.setCursor(0, 10);
        oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);
        oled.setCursor(85, 10);
        oled.printf("| FPS:%lu", 1000000 / (micros() - t));
        t = micros();

        oled.drawBox(0, 15, 128, 3);
        oled.setCursor(0, 30);
        oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
                    (int) roundf(dummy.currentJoints.a[0]), (int) roundf(dummy.currentJoints.a[1]),
                    (int) roundf(dummy.currentJoints.a[2]), (int) roundf(dummy.currentJoints.a[3]),
                    (int) roundf(dummy.currentJoints.a[4]), (int) roundf(dummy.currentJoints.a[5]));

        oled.drawBox(40, 35, 128, 24);
        oled.setFont(u8g2_font_6x12_tr);
        oled.setDrawColor(0);
        oled.setCursor(42, 45);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.X),
                    (int) roundf(dummy.currentPose6D.Y), (int) roundf(dummy.currentPose6D.Z));
        oled.setCursor(42, 56);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.A),
                    (int) roundf(dummy.currentPose6D.B), (int) roundf(dummy.currentPose6D.C));
        oled.setDrawColor(1);
        oled.setCursor(0, 45);
        oled.printf("[XYZ]:");
        oled.setCursor(0, 56);
        oled.printf("[ABC]:");

        oled.setFont(u8g2_font_10x20_tr);
        oled.setCursor(0, 78);
        if (dummy.IsEnabled())
        {
            for (int i = 1; i <= 6; i++)
                buf[i - 1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');
            buf[6] = 0;
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], buf);
        } else
        {
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], "======");
        }

        oled.sendBuffer();
    }
}


/* Timer Callbacks -------------------------------------------------------*/
void OnTimer7Callback()
{
    // TIM7 更新中断回调：唤醒固定周期控制线程并触发一次上下文切换。
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake & invoke thread IMMEDIATELY.
    // 在 ISR 中通过 vTaskNotifyGiveFromISR 通知任务，并根据 xHigherPriorityTaskWoken
    // 进行一次即时调度切换（portYIELD_FROM_ISR），确保控制环的实时性；
    // 注意：回调体保持轻量，不做任何复杂逻辑与浮点运算。
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* Default Entry -------------------------------------------------------*/
void Main(void)
{
    // 系统主入口：初始化通信、机器人、传感器、显示与用户线程，并启动控制定时器。
    // Init all communication staff, including USB-CDC/VCP/UART/CAN etc.
    // 1) 通信栈初始化：配置 USB/UART/CAN 流与对象树发布，建立上行/下行通道。
    InitCommunication();

    // Init Robot.
    // 2) 机器人初始化：加载参数、建立 CAN 会话、复位关节状态。
    dummy.Init();

    // Init IMU.
    // 3) 传感器初始化与自检：尝试初始化，等待 100ms 再重试，直到连接成功；随后初始化滤波器（采样/加速度/陀螺）。
    do
    {
        mpu6050.Init();
        osDelay(100);
    } while (!mpu6050.testConnection());
    mpu6050.InitFilter(200, 100, 50);

    // Init OLED 128x80.
    // 4) 显示与 PWM：OLED 准备完毕；启动四路 PWM 输出，默认占空比为 0。
    oled.Init();
    pwm.Start();

    // Init & Run User Threads.
    // 5) 用户线程创建：实时控制线程（优先级 Realtime）、命令处理与 OLED（Normal）。
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopFixUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityRealtime,
    };
    controlLoopFixUpdateHandle = osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                                             &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
        .name = "ControlLoopUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,
    };
    ControlLoopUpdateHandle = osThreadNew(ThreadControlLoopUpdate, nullptr,
                                          &ControlLoopUpdateTask_attributes);

    const osThreadAttr_t oledTask_attributes = {
        .name = "OledTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    oledTaskHandle = osThreadNew(ThreadOledUpdate, nullptr, &oledTask_attributes);

    // Start Timer Callbacks.
    // 6) 控制定时器：绑定 TIM7 回调并启动，当前节拍 200Hz（5ms），驱动固定周期控制。
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    // System started, light switch-led up.
    // 7) 开机提示与初始占空比：通过 UART4 输出剩余堆内存，点亮占空比以示系统启动。
    Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    pwm.SetDuty(PWM::CH_A1, 0.5);
}
