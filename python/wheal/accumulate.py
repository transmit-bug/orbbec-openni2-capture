"""多帧时间累积 —— 降噪与填洞，通用的一步。

**这里不含任何风团概念。** 输入是若干深度帧，输出是一张累积后的深度图加覆盖率
统计。两个消费者都从这里出发，彼此互不依赖：

- :mod:`wheal.geometry` —— 2.5D 建模（深度值本身就是模型）；
- :mod:`wheal.analyze` —— 风团测量（在累积结果上再拟合一个基准面）。

从 ``analyze`` 里搬出来的原因就是这个依赖方向：只要累积住在 ``analyze.py``，
任何通用 2.5D 工作都得 import 风团分析模块。

累积是本流程里最实在的一步：结构光匹配失败造成的空洞是**逐帧重掷**的
（实测相邻帧空洞掩码 IoU 仅 0.497），所以多帧能填；实测 120 帧把覆盖率从
43% 提到 94%（2.19 倍）。但**降噪远达不到 1/√N**——噪声是重尾的（
``std / (MAD×1.4826) ≈ 1.9``），60 帧只把噪声降到 1/1.99 而不是 1/7.7。
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
from typing import Literal

import numpy as np

from .frames import DepthFrame, Roi, valid_mask
from .stats import MAD_TO_SIGMA

__all__ = ["AccumulateParams", "FusedDepth", "accumulate"]


@dataclass(frozen=True)
class AccumulateParams:
    """时间累积的全部可调参数。

    Attributes:
        average_mode: 累积方式。**默认 ``trimmed_mean``，这是刻意的。**

            传感器输出整数毫米（``PIXEL_FORMAT_DEPTH_1_MM``），1 mm 的量化台阶
            与待测的隆起同量级（0.5–1.5 mm）。这带来一个不容易察觉的后果：

            - ``median`` **无法突破量化台阶**——量化值的中值仍是量化值。累积再多
              帧，噪声底也会假性归零，看起来"极准"，实则什么都没测到。
            - ``mean`` 可以恢复亚量子精度，靠传感器噪声充当抖动（dither），
              让样本跨越量化边界——但这也是裸均值最脆弱的地方。
            - 而裸均值会被结构光在毛发/边缘/反光处的**稀疏大离群值**污染。

            因此默认取 ``trimmed_mean``：先按 MAD 剔除时间维离群值，再对保留样本
            求均值，同时拿到稳健性与亚量子精度。

            ``median`` 保留作为对照手段；若报告里看到中值噪声底恒为 0，那不是
            精度好，而是量化台阶没有被突破。
        trim_k: ``trimmed_mean`` 的剔除倍数（MAD 单位）。
        max_valid_mm: 深度上限。超过即视为不可信。工作距离约 600 mm 时，
            1500 mm 足以排除远距离噪声而不误伤。``None`` 表示不设上限。
        quantum_mm: 传感器量化台阶。``PIXEL_FORMAT_DEPTH_1_MM`` 下为 1.0。
            仅用于给离群尺度一个诚实的**下限**（见 :func:`_trimmed_mean`）——
            量化数据未充分抖动时逐像素 MAD 会退化为 0，没有这个下限，
            ``trimmed_mean`` 会静默退化成裸均值。
    """

    average_mode: Literal["trimmed_mean", "median", "mean"] = "trimmed_mean"
    trim_k: float = 3.0
    max_valid_mm: float | None = 1500.0
    quantum_mm: float = 1.0


@dataclass(frozen=True)
class FusedDepth:
    """一次累积的结果与诊断信息。"""

    depth_mm: np.ndarray
    valid: np.ndarray
    frame_count: int
    per_frame_coverage: float
    fused_coverage: float

    @property
    def hole_fill_gain(self) -> float:
        """累积带来的覆盖率提升倍数。多帧填洞是本流程最显著的收益。"""
        if self.per_frame_coverage <= 0:
            return float("nan")
        return self.fused_coverage / self.per_frame_coverage

    def statistics(self) -> dict[str, float]:
        values = self.depth_mm[np.isfinite(self.depth_mm)]
        return {
            "frame_count": self.frame_count,
            "per_frame_coverage": self.per_frame_coverage,
            "fused_coverage": self.fused_coverage,
            "coverage_gain": self.hole_fill_gain,
            "depth_min_mm": float(values.min()) if values.size else float("nan"),
            "depth_median_mm": float(np.median(values)) if values.size else float("nan"),
            "depth_max_mm": float(values.max()) if values.size else float("nan"),
        }


def _trimmed_mean(stack: np.ndarray, trim_k: float, quantum_mm: float) -> np.ndarray:
    """先按 MAD 剔除时间维离群值，再对保留样本求均值。

    为什么不能用中值：传感器量化台阶为 1 mm，与待测高度同量级。量化样本的中值
    仍然是量化的，因此无论累积多少帧都无法突破台阶——噪声底会假性归零，而实测
    中值会把 0.4 mm 的真实隆起报成 1.0 mm（量到台阶上了）。

    为什么不能直接用均值：结构光在毛发、边缘、反光处的失效是**稀疏的大离群值**，
    单帧坏点就能把均值拉偏。

    关键细节：离群尺度的估计必须以**一个量化台阶**兜底，而不是它的 RMS。

    量化数据在未充分抖动时逐像素 MAD 会退化为 0。若此时直接用 ``trim_k × 0 = 0``
    作门限，所有离群值都会原样通过，``trimmed_mean`` 就静默退化成了裸均值。
    但兜底值不能取量化 RMS（``q/sqrt(12) = 0.289``）：``3 × 0.289 = 0.87`` 恰好
    **小于一个台阶**，于是把跨越到相邻整数的抖动样本（偏差 = 1.0）也当作离群值剔掉，
    结果又退回量化台阶（实测峰值锁在 1.0 mm）。丢掉的抖动恰恰是突破量化的机制。

    取 ``scale >= q`` 则 ``limit ≈ 3q``：真实的量化抖动（偏差半个到一个台阶）保留，
    而毛发/反光产生的数十至数百毫米跳变照旧剔除。

    用 ``np.fmax`` 而非 ``np.maximum``，因为全无效像素的 MAD 是 ``NaN``。
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        center = np.nanmedian(stack, axis=0)
        # |stack - center| 算一次就够了：MAD 要它，下面的剔除也要它。
        # 单独再算一遍会把这一项的峰值内存翻倍（300 帧全画幅约 370 MB）。
        deviation = np.abs(stack - center)
        mad = np.nanmedian(deviation, axis=0)

    scale = np.fmax(MAD_TO_SIGMA * mad, quantum_mm)
    limit = trim_k * scale
    keep = np.isfinite(stack) & (deviation <= limit[None, :, :])

    count = keep.sum(axis=0)
    total = np.where(keep, stack, 0.0).sum(axis=0, dtype=np.float64)
    return np.where(count > 0, total / np.maximum(count, 1), np.nan)


def accumulate(
    frames: list[DepthFrame] | tuple[DepthFrame, ...],
    roi: Roi | None = None,
    params: AccumulateParams | None = None,
) -> FusedDepth:
    """累积多帧，得到降噪且填洞后的深度图。

    ``roi`` 为 ``None`` 时使用整帧。

    覆盖率统计有两个，都很容易读错，所以在这里定义清楚：

    - ``per_frame_coverage`` —— **样本**层面的有效率：所有 ``帧 × 像素`` 组合里
      有效样本占多少。不是"每帧平均有多少像素有效"（两者数值相同，但前者的
      说法才对应实现）。
    - ``fused_coverage`` —— **像素**层面：累积后至少有一个可信样本的像素占多少。

    两者用**同一套**有效性定义（含 ``max_valid_mm`` 距离上限）。早期版本里
    前者不含距离上限、后者含，工作距离正常时看不出差别，一旦设了 ``max_valid_mm``
    就会给出两个口径混用的比值。

    Returns:
        :class:`FusedDepth`。``depth_mm`` 为 ``float64``，无有效样本处为 ``NaN``。

    内存提示：需要完整栈，开销约 ``N × h × w × 4`` 字节。300 帧全画幅约
    370 MB；把 ROI 收敛到目标周围可以把这一项降一到两个数量级。
    """
    frame_list = list(frames)
    if not frame_list:
        raise ValueError("frames 为空")
    height, width = frame_list[0].shape
    roi = roi or Roi(0, 0, width, height)
    params = params or AccumulateParams()

    stack = np.empty((len(frame_list), roi.h, roi.w), dtype=np.float32)
    for index, frame in enumerate(frame_list):
        stack[index] = roi.crop(frame.depth_mm)

    usable = valid_mask(stack, params.max_valid_mm)
    stack[~usable] = np.nan

    with warnings.catch_warnings():
        # 全无效像素会产生 all-NaN 切片警告；这是预期输入，不是异常。
        warnings.simplefilter("ignore", RuntimeWarning)
        if params.average_mode == "median":
            depth = np.nanmedian(stack, axis=0, overwrite_input=True)
        elif params.average_mode == "mean":
            depth = np.nanmean(stack, axis=0)
        else:
            depth = _trimmed_mean(stack, params.trim_k, params.quantum_mm)

    depth = np.asarray(depth, dtype=np.float64)
    valid = np.isfinite(depth)
    return FusedDepth(
        depth_mm=depth,
        valid=valid,
        frame_count=len(frame_list),
        per_frame_coverage=float(usable.mean()) if usable.size else 0.0,
        fused_coverage=float(valid.mean()) if valid.size else 0.0,
    )
