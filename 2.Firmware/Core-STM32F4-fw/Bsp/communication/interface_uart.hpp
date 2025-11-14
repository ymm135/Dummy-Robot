#ifndef __INTERFACE_UART_HPP
#define __INTERFACE_UART_HPP
/*
 * 基础特性概览（UART 接口）
 * - 职责：UART4/5 的协议输出通道与服务器任务启动入口。
 * - 设计：暴露 FreeRTOS 线程句柄，任务层处理解析与回传；ISR/DMA 仅搬运数据。
 * - StreamSink：通过 Fibre StreamSink 统一封装输出，避免 printf 全通道广播。
 * - 用法：调用 StartUartServer 启动服务并绑定串口流。
 */

#ifdef __cplusplus

#include "fibre/protocol.hpp"

extern StreamSink *uart4StreamOutputPtr;
extern StreamSink *uart5StreamOutputPtr;

extern "C" {
#endif

#include <cmsis_os.h>

extern osThreadId uart_thread;

void StartUartServer(void);

#ifdef __cplusplus
}
#endif

#endif // __INTERFACE_UART_HPP
