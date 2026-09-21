"""CJK 字体探测 —— 所有绘图模块共用。

单独成模块的原因：matplotlib 自带的 DejaVu Sans 不含汉字，直接画中文标签会得到
一串豆腐块，而它**只弹一个 UserWarning、不报错**。这种失败很容易被当成正常产出
（图上是一片方块，但脚本退出码是 0）。所以这个判断必须是显式的、且只做一次，
让每个画图的地方都能走同一套"有字体用中文、没字体退英文"的规则，而不是各自
复制一份候选名单然后慢慢走样。
"""

from __future__ import annotations

_CJK_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "Source Han Sans SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
)


def configure_cjk_font() -> bool:
    """尝试启用 CJK 字体。找不到时返回 ``False``，图表改走英文标签。"""
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    available = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in _CJK_CANDIDATES:
        if candidate in available:
            plt.rcParams["font.sans-serif"] = [candidate, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


#: 本机是否存在可用的 CJK 字体。在导入时确定一次，之后全局一致。
HAS_CJK = configure_cjk_font()


def t(zh: str, en: str) -> str:
    """有 CJK 字体就用中文标签，否则退到英文。"""
    return zh if HAS_CJK else en
