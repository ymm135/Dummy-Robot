"""
串口传输模块：
- 提供 StreamSource/StreamSink 与 PacketSource/PacketSink 的串口实现；
- 负责端口扫描与通道创建（discover_channels），以及基础读写（SerialStreamTransport）。
"""

import os
import re
import time
import traceback
import serial
import serial.tools.list_ports
import fibre
from fibre.utils import TimeoutError

# TODO: make this customizable
# 默认波特率：115200，需与固件端一致；如需更改，请同步设备端配置。
DEFAULT_BAUDRATE = 115200

class SerialStreamTransport(fibre.protocol.StreamSource, fibre.protocol.StreamSink):
    def __init__(self, port, baud):
        self._dev = serial.Serial(port, baud, timeout=1)
        # 串口打开：设置读超时 1s；写操作由 process_bytes 直通。

    def process_bytes(self, bytes):
        self._dev.write(bytes)
        # 写入字节流：上层 packet sink 会将包转为字节流后调用此函数。

    def get_bytes(self, n_bytes, deadline):
        """
        Returns n bytes unless the deadline is reached, in which case the bytes
        that were read up to that point are returned. If deadline is None the
        function blocks forever. A deadline before the current time corresponds
        to non-blocking mode.
        """
        if deadline is None:
            self._dev.timeout = None
        else:
            self._dev.timeout = max(deadline - time.monotonic(), 0)
        return self._dev.read(n_bytes)
        # 读操作：根据 deadline 计算剩余等待时间；None 表示阻塞读。

    def get_bytes_or_fail(self, n_bytes, deadline):
        result = self.get_bytes(n_bytes, deadline)
        if len(result) < n_bytes:
            raise TimeoutError("expected {} bytes but got only {}", n_bytes, len(result))
        return result
        # 保证读取足够字节，否则抛出超时错误。

    def close(self):
        self._dev.close()
        # 关闭串口：由通道断开回调触发，避免资源泄露。


def find_dev_serial_ports():
    try:
        return ['/dev/' + x for x in os.listdir('/dev')]
    except FileNotFoundError:
        return []
    # 在类 Unix 系统下，从 /dev 枚举潜在串口设备。

def find_pyserial_ports():
    return [x.device for x in serial.tools.list_ports.comports()]
    # 通过 pyserial API 获取可用串口列表（跨平台）。


def discover_channels(path, serial_number, callback, cancellation_token, channel_termination_token, logger):
    """
    Scans for serial ports that match the path spec.
    This function blocks until cancellation_token is set.
    Channels spawned by this function run until channel_termination_token is set.
    """
    if path == None:
        # This regex should match all desired port names on macOS,
        # Linux and Windows but might match some incorrect port names.
        regex = r'^(/dev/tty\.usbmodem.*|/dev/ttyACM.*|COM[0-9]+)$'
    else:
        regex = "^" + path + "$"
    # 端口匹配规则：
    # - 默认匹配 macOS 的 /dev/tty.usbmodem*、Linux 的 /dev/ttyACM* 与 Windows 的 COMx；
    # - 若指定 path，则严格按正则匹配（支持精确端口如 serial:COM3）。

    known_devices = []
    def device_matcher(port_name):
        if port_name in known_devices:
            return False
        return bool(re.match(regex, port_name))
    # 设备去重与匹配：避免重复打开同一端口。

    def did_disconnect(port_name, device):
        device.close()
        # TODO: yes there is a race condition here in case you wonder.
        known_devices.pop(known_devices.index(port_name))
        # 断开处理：关闭串口并从已知列表移除（存在竞争，足够满足基本场景）。

    while not cancellation_token.is_set():
        all_ports = find_pyserial_ports() + find_dev_serial_ports()
        new_ports = filter(device_matcher, all_ports)
        for port_name in new_ports:
            try:
                serial_device = SerialStreamTransport(port_name, DEFAULT_BAUDRATE)
                input_stream = fibre.protocol.PacketFromStreamConverter(serial_device)
                output_stream = fibre.protocol.StreamBasedPacketSink(serial_device)
                channel = fibre.protocol.Channel(
                        "serial port {}@{}".format(port_name, DEFAULT_BAUDRATE),
                        input_stream, output_stream, channel_termination_token, logger)
                channel.serial_device = serial_device
            except serial.serialutil.SerialException:
                logger.debug("Serial device init failed. Ignoring this port. More info: " + traceback.format_exc())
                known_devices.append(port_name)
            else:
                known_devices.append(port_name)
                channel._channel_broken.subscribe(lambda: did_disconnect(port_name, serial_device))
                callback(channel)
        time.sleep(1)
    # 发现循环：每秒扫描一次；成功创建通道后订阅断开事件并回调上层。
