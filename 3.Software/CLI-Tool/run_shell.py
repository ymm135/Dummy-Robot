#!/usr/bin/env python3
# 说明：
# 这是 Dummy-Robot 的命令行工具入口。支持：
# - shell：进入交互式 Python/IPython 控制台，动态发现并绑定设备变量（如 ref0）。
# - liveplotter：实时绘图示例（如编码器位置）。
# - drv-status / rate-test / udev-setup / generate-code / backup-config / restore-config：
#   基于 ref_tool 的实用功能命令。
#
# 参数说明：
# - --path：设备发现路径规格，支持逗号组合，如 'usb,serial:COM3'。
# - --serial-number：限定 12 位十六进制序列号匹配的设备。
# - --verbose：开启日志细节；--version：显示版本。
# - shell 子命令支持 --no-ipython：强制使用原生 Python 控制台。
#
# 设计要点：
# - print 函数封装：自动 flush，避免交互式输出阻塞。
# - Logger 与 Event：用于日志与跨线程退出信号（app_shutdown_token）。
# - sys.path 注入：兼容历史结构，将 fibre/python 优先入路径（见下方注释）。
from __future__ import print_function

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))),
    "Firmware", "fibre", "python"))
# 注意：此处向 sys.path 注入 Firmware/fibre/python 为历史兼容；
# 当前仓库已包含 CLI-Tool/fibre，Python 的 import 优先按 sys.path 顺序生效，
# 若 Firmware 路径不存在，仍可正常从 CLI-Tool/fibre 加载模块。
from fibre import Logger, Event
import ref_tool
from ref_tool.configuration import *

old_print = print


def print(*args, **kwargs):
    kwargs.pop('flush', False)
    old_print(*args, **kwargs)
    file = kwargs.get('file', sys.stdout)
    file.flush() if file is not None else sys.stdout.flush()
# 交互式打印封装：始终刷新输出，避免缓冲导致提示延迟。


script_path = os.path.dirname(os.path.realpath(__file__))

## Parse arguments ##
parser = argparse.ArgumentParser(description='Robot-Embedded-Framework command line utility\n',
                                 formatter_class=argparse.RawTextHelpFormatter)
subparsers = parser.add_subparsers(help='sub-command help', dest='command')
shell_parser = subparsers.add_parser('shell',
                                     help='Drop into an interactive python shell that lets you interact with the ODrive(s)')
# shell：进入交互式控制台，动态发现设备并挂载变量（由 fibre/ref_tool 实现）。
shell_parser.add_argument("--no-ipython", action="store_true",
                          help="Use the regular Python shell "
                               "instead of the IPython shell, "
                               "even if IPython is installed.")
subparsers.add_parser('liveplotter', help="For plotting of REF parameters (i.e. position) in real time")
# liveplotter：实时绘图工具，演示如何从设备拉取数据并绘制。

# General arguments
parser.add_argument("-p", "--path", metavar="PATH", action="store",
                    help="The path(s) where REF-board(s) should be discovered.\n"
                         "By default the script will connect to any REF-board on USB.\n\n"
                         "To select a specific USB device:\n"
                         "  --path usb:BUS:DEVICE\n"
                         "usbwhere BUS and DEVICE are the bus and device numbers as shown in `lsusb`.\n\n"
                         "To select a specific serial port:\n"
                         "  --path serial:PATH\n"
                         "where PATH is the path of the serial port. For example \"/dev/ttyUSB0\".\n"
                         "You can use `ls /dev/tty*` to find the correct port.\n\n"
                         "You can combine USB and serial specs by separating them with a comma (no space!)\n"
                         "Example:\n"
                         "  --path usb,serial:/dev/ttyUSB0\n"
                         "means \"discover any USB device or a serial device on /dev/ttyUSB0\"")
# PATH 语法：支持 'usb'、'serial:COMx' 或 '/dev/ttyUSBx' 等，多个用逗号连接。
parser.add_argument("-s", "--serial-number", action="store",
                    help="The 12-digit serial number of the device. "
                         "This is a string consisting of 12 upper case hexadecimal "
                         "digits as displayed in lsusb. \n"
                         "    example: 385F324D3037\n"
                         "You can list all devices connected to USB by running\n"
                         "(lsusb -d 1209:0d32 -v; lsusb -d 0483:df11 -v) | grep iSerial\n"
                         "If omitted, any device is accepted.")
parser.add_argument("-v", "--verbose", action="store_true",
                    help="print debug information")
parser.add_argument("--version", action="store_true",
                    help="print version information and exit")

parser.set_defaults(path="usb")
args = parser.parse_args()

# Default command
if args.command is None:
    args.command = 'shell'
    args.no_ipython = False
logger = Logger(verbose=args.verbose)

app_shutdown_token = Event()
# 统一退出令牌：在 finally 中设置，通知所有后台线程（设备发现/通信）结束。

try:
    if args.command == 'shell':
        # if ".dev" in ref_tool.__version__:
        #     print("")
        #     logger.warn("Developer Preview")
        #     print("")
        import ref_tool.shell

        ref_tool.shell.launch_shell(args, logger, app_shutdown_token)
        # 交互式控制台：内部调用 fibre.launch_shell，实现设备发现与变量绑定。

    elif args.command == 'liveplotter':
        from ref_tool.utils import start_liveplotter

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)

        # If you want to plot different values, change them here.
        # You can plot any number of values concurrently.
        cancellation_token = start_liveplotter(lambda: [
            ref_unit.axis0.encoder.pos_estimate,
            ref_unit.axis1.encoder.pos_estimate,
        ])
        # 修改上述列表即可绘制不同参数；支持多曲线并行绘制。

        print("Showing plot. Press Ctrl+C to exit.")
        while not cancellation_token.is_set():
            time.sleep(1)

    elif args.command == 'drv-status':
        from ref_tool.utils import print_drv_regs

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)
        print_drv_regs("Motor 0", ref_unit.axis0.motor)
        print_drv_regs("Motor 1", ref_unit.axis1.motor)
        # 打印驱动寄存器状态，便于诊断驱动芯片配置与故障。

    elif args.command == 'rate-test':
        from ref_tool.utils import rate_test

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)
        rate_test(ref_unit)
        # 传输速率测试：评估通道吞吐与延迟表现。

    elif args.command == 'udev-setup':
        from ref_tool.version import setup_udev_rules

        setup_udev_rules(logger)
        # 在 Linux 环境下安装 udev 规则，简化设备权限和枚举。

    elif args.command == 'generate-code':
        from ref_tool.code_generator import generate_code

        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     channel_termination_token=app_shutdown_token)
        generate_code(ref_unit, args.template, args.output)
        # 代码生成：根据设备对象模型与模板生成绑定代码。

    elif args.command == 'backup-config':
        from ref_tool.configuration import backup_config

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)
        backup_config(ref_unit, args.file, logger)
        # 备份设备参数到文件，便于迁移与快速恢复。

    elif args.command == 'restore-config':
        from ref_tool.configuration import restore_config

        print("Waiting for ODrive...")
        ref_unit = ref_tool.find_any(path=args.path, serial_number=args.serial_number,
                                     search_cancellation_token=app_shutdown_token,
                                     channel_termination_token=app_shutdown_token)
        restore_config(ref_unit, args.file, logger)
        # 从文件恢复设备参数，注意设备固件版本兼容性。

    else:
        raise Exception("unknown command: " + args.command)

except OperationAbortedException:
    logger.info("Operation aborted.")
finally:
    app_shutdown_token.set()
    # 统一通知后台任务结束（避免线程泄漏），保障进程平滑退出。
