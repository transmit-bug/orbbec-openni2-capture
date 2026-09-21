"""风团分析参数与结果类型。

单位约定：**全部为毫米（mm）**。传感器本身输出 ``PIXEL_FORMAT_DEPTH_1_MM``，
全程不做米制转换，避免引入转换错误。

时间累积的参数不在这里，在 :mod:`wheal.accumulate`——那是通用的一步。
:class:`AnalyzeParams` 继承它，所以“风团参数”可以直接当“累积参数”传给累积，
不需要任何字段映射（见类文档）。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .accumulate import AccumulateParams
from .frames import Roi

#: 均匀量化台阶的 RMS 值（标准差）。稳健拟合在量化支配下会退化为 0，
#: 此时这个值才是诚实的噪声底下限。
QUANTUM_RMS_FACTOR = 1.0 / 12.0**0.5


@dataclass(frozen=True)
class AnalyzeParams(AccumulateParams):
    """一次风团分析的全部可调参数。

    **继承 ``AccumulateParams``**，而不是把它的字段复制一份。这不是省事，是一个
    建模声明：“一次风团分析所用的参数，**是**一种时间累积参数，再加风团专属的东西。”

    两种替代方案都不如继承：

    - 把累积字段平铺在后面 —— 字段名重复两份，改一处忘一处，而两者的默认值
      必须永远一致；
    - 把累积参数当子字段组合（``params.accumulate.average_mode``）—— 语义上更
      “干净”，但调用端变成 ``replace(params, accumulate=replace(...))``，“改变平均
      方式”这么一个平绕动作要穿过两层对象。

    继承同时让 ``params`` 可以直接传给 :func:`wheal.accumulate.accumulate`，
    不需要一个字段映射函数（那正是最容易写错、且错了不会报错的地方）。

    Attributes:
        baseline_degree: 基准面多项式阶数。2 = 二次曲面，足以刻画前臂的缓慢弯曲。
        robust_k: 稳健剔除倍数。残差超过 ``robust_k × sigma`` 的像素被视为离群
            （风团本身、毛发、飞点），不参与基准面拟合。
        height_threshold_mm: 计算面积与体积时的高度阈值。``None`` 表示自动取
            ``robust_k × noise_floor``，避免写死一个与噪声底脱节的魔数。
        flying_pixel_jump_mm: 飞点判据。3×3 邻域内深度极差超过该值即判为深度
            不连续处的飞点。``None`` 关闭。
        lateral_scale_mm_per_px: 横向尺度，用于把像素数换算成 mm²/mm³。
            **默认 None**：隆起高度是纯 Z 量，不需要内参；但面积与体积需要横向
            尺度，缺失时对应指标返回 ``None`` 而不是编造一个数字。
        max_iterations: 基准面稳健重拟合的最大迭代次数。

    时间累积相关字段（``average_mode`` / ``trim_k`` / ``max_valid_mm`` /
    ``quantum_mm``）继承自父类，各自含义见 :class:`~wheal.accumulate.AccumulateParams`。
    """

    baseline_degree: int = 2
    robust_k: float = 3.0
    height_threshold_mm: float | None = None
    flying_pixel_jump_mm: float | None = 20.0
    lateral_scale_mm_per_px: float | None = None
    max_iterations: int = 8


@dataclass(frozen=True)
class WhealMetrics:
    """风团的量化指标。

    注意 **隆起量（elevation）** 与 **高度残差（height residual）** 的区别：
    后者是逐像素的差值场，前者是从它导出的标量指标（此处即 ``peak_height_mm``
    与 ``volume_mm3``）。
    """

    peak_height_mm: float
    noise_floor_mm: float
    valid_ratio: float
    measurable_ratio: float
    area_above_px: int
    threshold_mm: float
    area_above_mm2: float | None
    volume_mm3: float | None

    def as_dict(self) -> dict[str, float | int | None]:
        return {
            "peak_height_mm": self.peak_height_mm,
            "noise_floor_mm": self.noise_floor_mm,
            "valid_ratio": self.valid_ratio,
            "measurable_ratio": self.measurable_ratio,
            "area_above_px": self.area_above_px,
            "threshold_mm": self.threshold_mm,
            "area_above_mm2": self.area_above_mm2,
            "volume_mm3": self.volume_mm3,
        }


@dataclass(frozen=True)
class AnalysisResult:
    """``analyze`` 的返回值。

    退化输入（全无效帧、有效像素不足以拟合基准面）不会抛异常，而是返回一个
    各指标为 ``NaN`` 的结果并在 ``warnings`` 中说明原因。这样调用方（尤其是批量
    扫帧数的 sweep）可以确定地判断"这次没测出来"，而不是被异常打断。
    """

    height_map_mm: np.ndarray
    baseline_mm: np.ndarray
    valid_mask: np.ndarray
    measurable_mask: np.ndarray
    metrics: WhealMetrics
    roi: Roi
    frame_count: int
    warnings: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        # 冗余但必要：可复现性依赖数据的不变性，防止调用方就地修改高度图。
        for name in ("height_map_mm", "baseline_mm"):
            array = getattr(self, name)
            if array.flags.writeable:
                array.setflags(write=False)

    @property
    def ok(self) -> bool:
        """是否得到了数值可用的高度测量。

        ``warnings`` 只是附加说明（例如噪声底被量化下限截断），**不算失败**。
        真正的失败是退化输入：峰值高度为 ``NaN``，或没有任何可测像素。

        这两者必须分开，否则单帧的曲线点会因为"有告警"而变成 ``NaN``，
        把"单帧分辨不出 0.8 mm"这个**有用的结论**藏起来。
        """
        return bool(np.isfinite(self.metrics.peak_height_mm) and self.metrics.measurable_ratio > 0)


def quantisation_floor_mm(quantum_mm: float = 1.0, n_samples: int = 1) -> float:
    """N 帧量化数据平均后能达到的噪声底下限。

    单帧均匀量化的 RMS 是 ``q/sqrt(12)``；以传感器噪声充当抖动（dither）平均 N 帧，
    最乐观也只能降到 ``q/sqrt(12N)``。

    **低于这个值的稳健 sigma 不是精度，是拟合的浮点残渣。** 实测中单帧的稳健
    sigma 会退化为 ``3.7e-12`` 这种量级（lstsq 的数值余项），若不设下限就会把
    "什么都没测到"报成"极准"。
    """
    return float(quantum_mm) * QUANTUM_RMS_FACTOR / max(int(n_samples), 1) ** 0.5


def min_resolvable_height_mm(noise_floor_mm: float, k: float = 3.0) -> float:
    """由噪声底估算可分辨的最小隆起高度。

    这是调研报告里那条曲线的纵轴：``k × sigma``。``k=3`` 是常用的检测门限，
    但在报告里应当同时给出实测的恢复实验值，而不是只靠这个解析估计。
    """
    return float(k * noise_floor_mm)
