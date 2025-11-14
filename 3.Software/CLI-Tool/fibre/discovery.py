"""
Fibre 设备发现：
- 提供扫描并构建远程对象的工具函数（find_all / find_any）；
- 通过多种传输通道（USB、串口、TCP、UDP）并行发现设备；
- 读取端点 0 的 JSON 接口定义以初始化对象树。
"""

import sys
import json
import time
import threading
import traceback
import fibre.protocol
import fibre.utils
import fibre.remote_object
from fibre.utils import Event, Logger
from fibre.protocol import ChannelBrokenException, TimeoutError

# 加载已安装的传输层：不同通道对应不同 discover_channels 实现。

channel_types = {}

try:
    import fibre.usbbulk_transport
    channel_types['usb'] = fibre.usbbulk_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.serial_transport
    channel_types['serial'] = fibre.serial_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.tcp_transport
    channel_types['tcp'] = fibre.tcp_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.udp_transport
    channel_types['udp'] = fibre.udp_transport.discover_channels
except ImportError:
    pass

def noprint(text):
    pass

def find_all(path, serial_number,
         did_discover_object_callback,
         search_cancellation_token,
         channel_termination_token,
         logger):
    """
    Starts scanning for Fibre nodes that match the specified path spec and calls
    the callback for each Fibre node that is found.
    This function is non-blocking.

    中文说明：
    - 根据 path 执行多通道并行扫描（逗号分隔支持多路并行）。
    - 通道建立后读取端点 0（JSON 接口定义）以创建 RemoteObject。
    - 若指定 serial_number，则仅回调匹配的设备对象。
    - 通过 cancellation_token 与 channel_termination_token 协调退出与资源释放。
    """

    def did_discover_channel(channel):
        """
        Inits an object from a given channel and then calls did_discover_object_callback
        with the created object
        This queries the endpoint 0 on that channel to gain information
        about the interface, which is then used to init the corresponding object.
        """
        # 从通道初始化对象：
        # 1) 读取端点 0 获取 ASCII JSON 接口定义；
        # 2) 计算 CRC16 保存到通道，用于接口一致性校验；
        # 3) 解析 JSON 并包装为 RemoteObject（根名 fibre_node）。
        try:
            logger.debug("Connecting to device on " + channel._name)
            try:
                json_bytes = channel.remote_endpoint_read_buffer(0)
            except (TimeoutError, ChannelBrokenException):
                logger.debug("no response - probably incompatible")
                return
            json_crc16 = fibre.protocol.calc_crc16(fibre.protocol.PROTOCOL_VERSION, json_bytes)
            channel._interface_definition_crc = json_crc16
            try:
                json_string = json_bytes.decode("ascii")
            except UnicodeDecodeError:
                logger.debug("device responded on endpoint 0 with something that is not ASCII")
                return
            logger.debug("JSON: " + json_string.replace('{"name"', '\n{"name"'))
            logger.debug("JSON checksum: 0x{:02X} 0x{:02X}".format(json_crc16 & 0xff, (json_crc16 >> 8) & 0xff))
            try:
                json_data = json.loads(json_string)
            except json.decoder.JSONDecodeError as error:
                logger.debug("device responded on endpoint 0 with something that is not JSON: " + str(error))
                return
            json_data = {"name": "fibre_node", "members": json_data}
            obj = fibre.remote_object.RemoteObject(json_data, None, channel, logger)

            obj.__dict__['_json_data'] = json_data['members']
            obj.__dict__['_json_crc'] = json_crc16

            device_serial_number = fibre.utils.get_serial_number_str(obj)
            if serial_number != None and device_serial_number != serial_number:
                logger.debug("Ignoring device with serial number {}".format(device_serial_number))
                return
            did_discover_object_callback(obj)
        except Exception:
            logger.debug("Unexpected exception after discovering channel: " + traceback.format_exc())

    # 按照 path 前缀为每种连接类型启动对应的发现线程。
    for search_spec in path.split(','):
        prefix = search_spec.split(':')[0]
        the_rest = ':'.join(search_spec.split(':')[1:])
        if prefix in channel_types:
            t = threading.Thread(target=channel_types[prefix],
                             args=(the_rest, serial_number, did_discover_channel, search_cancellation_token, channel_termination_token, logger))
            t.daemon = True
            t.start()
        else:
            raise Exception("Invalid path spec \"{}\"".format(search_spec))


def find_any(path="usb", serial_number=None,
        search_cancellation_token=None, channel_termination_token=None,
        timeout=None, logger=Logger(verbose=False)):
    """
    Blocks until the first matching Fibre node is connected and then returns that node

    中文说明：
    - 阻塞等待首个匹配设备并返回；
    - 使用 Event 协调等待与取消；
    - finally 中置位事件以通知 find_all 终止。
    """
    result = [ None ]
    done_signal = Event(search_cancellation_token)
    def did_discover_object(obj):
        result[0] = obj
        done_signal.set()
    find_all(path, serial_number, did_discover_object, done_signal, channel_termination_token, logger)
    try:
        done_signal.wait(timeout=timeout)
    finally:
        done_signal.set() # terminate find_all
    return result[0]
