
# Fibre 交互式 Shell：将已发现设备注入交互环境，支持 IPython。
# 设备接入后命名为 branding_short+序号，例如 dev0、dev1，可自定义。
import sys
import platform
import threading
import fibre

def did_discover_device(device,
                        interactive_variables, discovered_devices,
                        branding_short, branding_long,
                        logger, app_shutdown_token):
    """
    Handles the discovery of new devices by displaying a
    message and making the device available to the interactive
    console
    """
    # 设备接入：
    # - 根据序列号进行去重与索引，生成交互名如 dev0/dev1；
    # - 将设备注入到交互变量与 globals（便于 Tab 补全与快速访问）；
    # - 订阅通道断开事件以提示设备丢失。
    serial_number = '{:012X}'.format(device.serial_number) if hasattr(device, 'serial_number') else "[unknown serial number]"
    if serial_number in discovered_devices:
        verb = "Reconnected"
        index = discovered_devices.index(serial_number)
    else:
        verb = "Connected"
        discovered_devices.append(serial_number)
        index = len(discovered_devices) - 1
    interactive_name = branding_short + str(index)

    # Publish new device to interactive console
    interactive_variables[interactive_name] = device
    globals()[interactive_name] = device # Add to globals so tab complete works
    logger.notify("{} to {} {} as {}".format(verb, branding_long, serial_number, interactive_name))

    # Subscribe to disappearance of the device
    device.__channel__._channel_broken.subscribe(lambda: did_lose_device(interactive_name, logger, app_shutdown_token))

def did_lose_device(interactive_name, logger, app_shutdown_token):
    """
    Handles the disappearance of a device by displaying
    a message.
    """
    # 设备丢失：在未进入关闭流程时给出告警提示。
    if not app_shutdown_token.is_set():
        logger.warn("Oh no {} disappeared".format(interactive_name))

def launch_shell(args,
                interactive_variables,
                print_banner, print_help,
                logger, app_shutdown_token,
                branding_short="dev", branding_long="device"):
    """
    Launches an interactive python or IPython command line
    interface.
    As devices are connected they are made available as
    "dev0", "dev1", ...
    The names of the variables can be customized by setting branding_short.
    
    中文说明：
    - 启动交互式控制台（优先使用 IPython，未安装则回退到标准 Python）。
    - 设备发现后自动注入到交互环境（变量名由 branding_short 决定，如 dev0）。
    - 支持打印横幅与帮助，并在退出时置位关闭标志。
    """

    discovered_devices = []
    globals().update(interactive_variables)

    # 设备发现：根据 path/serial_number 开始扫描，发现后注入交互环境。
    logger.debug("Waiting for {}...".format(branding_long))
    fibre.find_all(args.path, args.serial_number,
                    lambda dev: did_discover_device(dev, interactive_variables, discovered_devices, branding_short, branding_long, logger, app_shutdown_token),
                    app_shutdown_token,
                    app_shutdown_token,
                    logger=logger)

    # 检查 IPython 安装状态：无则提示用户可安装以获得更好体验。
    if args.no_ipython:
        use_ipython = False
    else:
        try:
            import IPython
            use_ipython = True
        except:
            print("Warning: you don't have IPython installed.")
            print("If you want to have an improved interactive console with pretty colors,")
            print("you should install IPython\n")
            use_ipython = False

    interactive_variables["help"] = lambda: print_help(args, len(discovered_devices) > 0)

    # 交互环境：优先嵌入 IPython，否则使用标准 Python 交互（尽量开启 Tab 补全）。
    if use_ipython:
        help = lambda: print_help(args, len(discovered_devices) > 0) # Override help function # pylint: disable=W0612
        locals()['__name__'] = globals()['__name__'] # to fix broken "%run -i script.py"
        console = IPython.terminal.embed.InteractiveShellEmbed(banner1='')
        console.runcode = console.run_code # hack to make IPython look like the regular console
        interact = console
    else:
        # Enable tab complete if possible
        try:
            import readline # Works only on Unix
            readline.parse_and_bind("tab: complete")
        except:
            sudo_prefix = "" if platform.system() == "Windows" else "sudo "
            print("Warning: could not enable tab-complete. User experience will suffer.\n"
                "Run `{}pip install readline` and then restart this script to fix this."
                .format(sudo_prefix))

        import code
        console = code.InteractiveConsole(locals=interactive_variables)
        interact = lambda: console.interact(banner='')

    # 安装异常过滤钩子：隐藏 ChannelBrokenException，改善交互体验。
    console.runcode('import sys')
    console.runcode('superexcepthook = sys.excepthook')
    console.runcode('def newexcepthook(ex_class,ex,trace):\n'
                    '  if ex_class.__module__ + "." + ex_class.__name__ != "fibre.ChannelBrokenException":\n'
                    '    superexcepthook(ex_class,ex,trace)')
    console.runcode('sys.excepthook=newexcepthook')


    # 启动交互：打印横幅，进入交互；退出后置位关闭标志。
    print_banner()
    logger._skip_bottom_line = True
    interact()
    app_shutdown_token.set()
