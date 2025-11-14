import os
import sys

# 路径注入：将 Firmware/fibre/python 放入 sys.path 以便导入 fibre 包。
# 这样无需单独安装即可使用本仓库自带的 Fibre Python 实现。
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.realpath(__file__)))),
    "Firmware", "fibre", "python"))

# 语法糖：为用户直接导出 fibre.find_any/find_all，便于调用；
# 使用 try/except 以规避安装阶段的依赖问题。
try:
    import fibre

    find_any = fibre.find_any
    find_all = fibre.find_all
except:
    pass

# 版本约定：向包暴露 __version__ 字符串（由 version.py 计算）。
from .version import get_version_str

del get_version_str
