"""``analyze`` —— **Seam 1**，整个调研的核心，也是一个纯函数。

无 I/O、无全局状态、无随机性：同样的输入必然得到同样的输出。所有的量化结论
（噪声底、可分辨的最小隆起高度）都通过对这个函数反复调用来获得，因此它的
纯粹性不是洁癖，而是可复现性的前提。

内部两步（稳健基准面 → 残差与指标）刻意保持私有，不额外暴露接缝。
**累积不在这里**，在 :mod:`wheal.accumulate`：那是通用的一步，几何重建也用它。
本模块是它的消费者，不是它的拥有者。

**符号约定**：深度是到相机的距离，风团**朝相机隆起**，所以深度值更小。

    height_residual = baseline - depth      # 正值 = 隆起

这个符号如果搞反，风团会变成凹坑，而所有统计量仍然"看起来正常"。

本模块与 :mod:`wheal.geometry` 的关系是**包含**而非并列：``analyze``
= 累积 + 一个基准面。对着平墙跑 ``analyze``，``fit_baseline`` 会把墙拟合掉，
报出墙的粗糙度——它并不需要知道那是墙。
"""

from __future__ import annotations

import numpy as np
from scipy import ndimage

from .accumulate import FusedDepth, accumulate
from .baseline import BaselineFit, fit_baseline
from .frames import DepthFrame, Roi, SequenceMetadata
from .metrics import (
    AnalysisResult,
    AnalyzeParams,
    WhealMetrics,
    quantisation_floor_mm,
)

__all__ = ["analyze", "analyze_source"]


def _flying_pixel_mask(depth: np.ndarray, valid: np.ndarray, jump_mm: float) -> np.ndarray:
    """屏蔽深度不连续处的飞点。

    飞点在高度残差图上表现为沿轮廓的一圈虚假隆起，与风团特征高度相似，必须显式
    剔除。判据是 3×3 邻域内有效深度的极差。

    已知局限：邻域含有无效像素时无法判断，这些像素**不会被标记**。也就是说紧贴
    空洞边缘的飞点会被漏掉。这是刻意选择的保守行为——宁可漏报，也不要把真实的
    皮肤边缘误判成飞点。
    """
    local_max = _neighbourhood_extreme(depth, valid, fill=-np.inf, maximum=True)
    local_min = _neighbourhood_extreme(depth, valid, fill=np.inf, maximum=False)
    finite = np.isfinite(local_max) & np.isfinite(local_min)
    jump = np.where(finite, local_max - local_min, 0.0)
    return valid & (jump > jump_mm)


def _neighbourhood_extreme(
    depth: np.ndarray, valid: np.ndarray, *, fill: float, maximum: bool
) -> np.ndarray:
    """3×3 邻域极值，只统计有效像素。邻域全无效处返回非有限值。"""
    prepared = np.where(valid, depth, fill)
    if maximum:
        return ndimage.maximum_filter(prepared, size=3, mode="constant", cval=-np.inf)
    return ndimage.minimum_filter(prepared, size=3, mode="constant", cval=np.inf)


def analyze(
    frames: list[DepthFrame] | tuple[DepthFrame, ...],
    roi: Roi,
    params: AnalyzeParams | None = None,
) -> AnalysisResult:
    """从若干帧静态深度图测量风团的隆起量。

    Args:
        frames: 同一静止场景的若干帧。**必须静止**——这个函数不做配准，
            隐含假设是各帧对应同一坐标系（镜头不动、被试不动）。
        roi: 感兴趣区域。应覆盖风团**及其周围的正常皮肤**，否则基准面无约束。
        params: 分析参数。

    Returns:
        :class:`AnalysisResult`。退化输入不抛异常，而是给出 ``NaN`` 指标与
        ``warnings``。
    """
    params = params or AnalyzeParams()
    frame_list = list(frames)
    if not frame_list:
        raise ValueError("frames 为空：至少需要一帧才能分析")

    roi.check_within(frame_list[0].shape)

    # 累积是共用的那一步（见 wheal.accumulate）。这里只消费它的结果，
    # 不在本模块里重写一份——两个入口会慢慢长成两种行为。
    fused: FusedDepth = accumulate(frame_list, roi, params)
    depth, valid = fused.depth_mm, fused.valid
    valid_ratio = float(valid.mean()) if valid.size else 0.0

    if params.flying_pixel_jump_mm is not None:
        flying = _flying_pixel_mask(depth, valid, params.flying_pixel_jump_mm)
    else:
        flying = np.zeros_like(valid)
    measurable = valid & ~flying
    measurable_ratio = float(measurable.mean()) if measurable.size else 0.0

    shape = depth.shape
    nan_baseline = np.full(shape, np.nan, dtype=np.float64)
    nan_height = np.full(shape, np.nan, dtype=np.float64)

    notes: list[str] = []
    fit: BaselineFit | None = fit_baseline(
        depth,
        measurable,
        degree=params.baseline_degree,
        robust_k=params.robust_k,
        max_iterations=params.max_iterations,
    )

    if valid_ratio <= 0.0:
        notes.append("no valid depth pixels in ROI")
    if fit is None:
        notes.append(f"too few valid pixels for baseline degree={params.baseline_degree}")
        return _degenerate(
            nan_height,
            nan_baseline,
            valid,
            measurable,
            params,
            roi,
            len(frame_list),
            valid_ratio,
            measurable_ratio,
            notes,
        )

    baseline = fit.evaluate()
    # 正残差 = 皮肤比基准面更靠近相机 = 隆起。
    height = np.where(measurable, baseline - depth, np.nan)

    noise_floor = fit.residual_sigma_mm
    floor_mm = quantisation_floor_mm(params.quantum_mm, len(frame_list))
    if not np.isfinite(noise_floor) or noise_floor < floor_mm:
        # 稳健 sigma 低于量化下限，说明累积还没突破 1 mm 台阶，残差是拟合的浮点
        # 残渣而非真实噪声。报一个诚实的下限，而不是报 1e-12 —— 后者会被读成"极准"。
        notes.append(
            f"noise floor {noise_floor:.3e} mm is below the quantisation limit "
            f"{floor_mm:.4f} mm for {len(frame_list)} frame(s); reporting the limit"
        )
        noise_floor = floor_mm
    threshold = (
        params.height_threshold_mm
        if params.height_threshold_mm is not None
        else params.robust_k * noise_floor
    )

    finite = np.isfinite(height)
    if finite.any():
        peak = float(np.nanmax(height))
    else:
        peak = float("nan")
        notes.append("no measurable pixels after flying-pixel rejection")

    above = finite & (height > threshold)
    area_px = int(above.sum())
    if params.lateral_scale_mm_per_px is not None:
        area_per_px = float(params.lateral_scale_mm_per_px) ** 2
        area_mm2: float | None = area_px * area_per_px
        volume_mm3: float | None = float(np.nansum(np.where(above, height, 0.0))) * area_per_px
    else:
        # 没有横向尺度就不编造面积/体积。隆起高度本身不需要内参，这是刻意的简化。
        area_mm2 = None
        volume_mm3 = None

    metrics = WhealMetrics(
        peak_height_mm=peak,
        noise_floor_mm=noise_floor,
        valid_ratio=valid_ratio,
        measurable_ratio=measurable_ratio,
        area_above_px=area_px,
        threshold_mm=float(threshold),
        area_above_mm2=area_mm2,
        volume_mm3=volume_mm3,
    )
    return AnalysisResult(
        height_map_mm=height,
        baseline_mm=baseline,
        valid_mask=valid,
        measurable_mask=measurable,
        metrics=metrics,
        roi=roi,
        frame_count=len(frame_list),
        warnings=tuple(notes),
    )


def _degenerate(
    height: np.ndarray,
    baseline: np.ndarray,
    valid: np.ndarray,
    measurable: np.ndarray,
    params: AnalyzeParams,
    roi: Roi,
    frame_count: int,
    valid_ratio: float,
    measurable_ratio: float,
    notes: list[str],
) -> AnalysisResult:
    return AnalysisResult(
        height_map_mm=height,
        baseline_mm=baseline,
        valid_mask=valid,
        measurable_mask=measurable,
        metrics=WhealMetrics(
            peak_height_mm=float("nan"),
            noise_floor_mm=float("nan"),
            valid_ratio=valid_ratio,
            measurable_ratio=measurable_ratio,
            area_above_px=0,
            threshold_mm=params.height_threshold_mm or float("nan"),
            area_above_mm2=None,
            volume_mm3=None,
        ),
        roi=roi,
        frame_count=frame_count,
        warnings=tuple(notes),
    )


def analyze_source(
    source,
    roi: Roi,
    params: AnalyzeParams | None = None,
    *,
    limit: int | None = None,
) -> tuple[AnalysisResult, SequenceMetadata]:
    """便利包装：从 :class:`FrameSource`（Seam 2）读帧后交给 :func:`analyze`。

    刻意做得很薄——Seam 2 的价值在于把磁盘挡在外面，而不是引入新的分析逻辑。
    """
    frames = []
    for frame in source.frames():
        frames.append(frame)
        if limit is not None and len(frames) >= limit:
            break
    return analyze(frames, roi, params), source.metadata()
