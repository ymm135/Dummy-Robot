import fibre
import ref_tool
from ref_tool.utils import start_liveplotter

def print_banner():
    # 入口横幅：提醒连接设备，提供基本指令。
    print('Please connect your Dummy-Robot.')
    print('You can also type help() or quit().')

def print_help(args, have_devices):
    print('')
    if have_devices:
        # 帮助说明：连接到指定 path 并上电，成功后会打印连接提示。
        print('Connect your REF-Unit to {} and power it up.'.format(args.path))
        print('After that, the following message should appear:')
        print('  "Connected to REF-Unit [serial number] as odrv0"')
        print('')
        print('Once the REF-Unit is connected, type "ref0." and press <tab>')
    else:
        print('Type "ref0." and press <tab>')
    print('This will present you with all the properties that you can reference')
    print('')
    print('For example: "odrv0.motor0.encoder.pos_estimate"')
    print('will print the current encoder position on motor 0')
    print('and "odrv0.motor0.pos_setpoint = 10000"')
    print('will send motor0 to 10000')
    print('')


interactive_variables = {}

discovered_devices = []

def did_discover_device(odrive, logger, app_shutdown_token):
    """
    Handles the discovery of new devices by displaying a
    message and making the device available to the interactive
    console
    """
    # 设备接入：
    # - 通过序列号去重并分配索引，交互名形如 odrv0/odrv1；
    # - 注入到交互环境与 globals（支持 Tab 补全）；
    # - 订阅通道断开事件以提示设备丢失。
    serial_number = odrive.serial_number if hasattr(odrive, 'serial_number') else "[unknown serial number]"
    if serial_number in discovered_devices:
        verb = "Reconnected"
        index = discovered_devices.index(serial_number)
    else:
        verb = "Connected"
        discovered_devices.append(serial_number)
        index = len(discovered_devices) - 1
    interactive_name = "odrv" + str(index)

    # Publish new ODrive to interactive console
    interactive_variables[interactive_name] = odrive
    globals()[interactive_name] = odrive # Add to globals so tab complete works
    logger.notify("{} to ODrive {:012X} as {}".format(verb, serial_number, interactive_name))

    # Subscribe to disappearance of the device
    odrive.__channel__._channel_broken.subscribe(lambda: did_lose_device(interactive_name, logger, app_shutdown_token))

def did_lose_device(interactive_name, logger, app_shutdown_token):
    """
    Handles the disappearance of a device by displaying
    a message.
    """
    # 设备丢失：在未进入关闭流程时给出告警提示。
    if not app_shutdown_token.is_set():
        logger.warn("Oh no {} disappeared".format(interactive_name))

def launch_shell(args, logger, app_shutdown_token):
    """
    Launches an interactive python or IPython command line
    interface.
    As ODrives are connected they are made available as
    "odrv0", "odrv1", ...

    中文说明：
    - 启动品牌化的交互 Shell，注入工具函数 start_liveplotter；
    - 设备接入后以 odrvX 命名，便于快速操作（与历史示例一致）；
    - 通过 branding_* 参数控制文案与提示（品牌短名/长名）。
    """

    interactive_variables = {
        'start_liveplotter': start_liveplotter,
    }

    fibre.launch_shell(args,
                       interactive_variables,
                       print_banner, print_help,
                       logger, app_shutdown_token,
                       branding_short="dummy", branding_long="Dummy-Robot")
    # 复用通用 shell：设置品牌短名与长名用于横幅与提示输出。
