#ifndef __INTERFACE_USB_HPP
#define __INTERFACE_USB_HPP
/*
 * 基础特性概览（USB 接口）
 * - 职责：USB CDC 通道服务器与统计信息管理。
 * - USBStats_t：维护收发计数与 TX 过载，便于流控与诊断。
 * - 入口：usb_rx_process_packet 解析包；StartUsbServer 启动任务与流。
 * - 模型：采用 FreeRTOS 任务处理，ISR 负责延迟处理/唤醒，保持控制环平稳。
 */

#ifdef __cplusplus

#include "fibre/protocol.hpp"

extern StreamSink *usbStreamOutputPtr;

extern "C" {
#endif

#include <cmsis_os.h>
#include <stdint.h>

typedef struct
{
    uint32_t rx_cnt;
    uint32_t tx_cnt;
    uint32_t tx_overrun_cnt;
} USBStats_t;

extern USBStats_t usb_stats_;

void usb_rx_process_packet(uint8_t *buf, uint32_t len, uint8_t endpoint_pair);
void StartUsbServer(void);

#ifdef __cplusplus
}
#endif

#endif // __INTERFACE_USB_HPP
