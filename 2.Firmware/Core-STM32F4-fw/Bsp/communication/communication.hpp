#ifndef COMMANDS_H
#define COMMANDS_H
/*
 * 基础特性概览（通信栈）
 * - 职责：统一初始化通信栈、注册 Fibre 对象树、调度通信任务。
 * - 支持接口：USB CDC、UART4/5、CAN1/2；采用 FreeRTOS 任务与信号量解耦 ISR。
 * - 任务说明：CommunicationTask 负责协议收发与调度；UsbDeferredInterruptTask 处理 USB IRQ 延迟。
 * - 提交协议：COMMIT_PROTOCOL 宏在静态缓冲上构建对象树并发布，避免动态分配碎片。
 * - 时序策略：解析/编码在任务中执行；ISR 仅做标志/唤醒，保证 100Hz 控制周期稳定。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <cmsis_os.h>

void InitCommunication(void);
void CommitProtocol();
void CommunicationTask(void *ctx);
void UsbDeferredInterruptTask(void *ctx);

#ifdef __cplusplus
}

#include <functional>
#include <limits>
#include "ascii_processor.hpp"
#include "interface_usb.hpp"
#include "interface_uart.hpp"
#include "interface_can.hpp"

#define COMMIT_PROTOCOL \
using treeType = decltype(MakeObjTree());\
uint8_t treeBuffer[sizeof(treeType)];\
void CommitProtocol()\
{\
    auto treePtr = new(treeBuffer) treeType(MakeObjTree());\
    fibre_publish(*treePtr);\
}\


#endif
#endif /* COMMANDS_H */
