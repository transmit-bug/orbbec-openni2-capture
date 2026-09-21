"""扫帧数与可分辨高度 —— 调研报告的核心曲线。

这是 issue #1 要求的主要产出：

- **噪声底 vs 帧数**：时间累积把随机噪声降多少。
- **可分辨的最小隆起高度 vs 帧数**：据此决定要采多少帧。

两个来源刻意分开：

- :func:`noise_floor_curve` —— 对真实录制序列做前缀切片，给出实测曲线。
- :func:`resolve_limit` —— 在**合成**数据上注入已知高度的凸起，找出能被准确
  恢复的最小高度。这是经验证的量，而不是从噪声底反推的解析估计。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .analyze import analyze
from .frames import DepthFrame, Roi
from .metrics import AnalyzeParams
from .synthetic import SyntheticSpec, ground_truth_height_mm
from .synthetic import frames as synthetic_frames


@dataclass(frozen=True)
class CurvePoint:
    """扫帧数曲线上的一个点。"""

    frame_count: int
    noise_floor_mm: float
    min_resolvable_mm: float
    peak_height_mm: float
    valid_ratio: float
    ok: bool


def default_frame_counts(total: int) -> list[int]:
    """几何级数帧数，从 1 到 ``total``，避免线性序列浪费点数。"""
    if total <= 0:
        return []
    counts = []
    value = 1
    while value < total:
        counts.append(value)
        value *= 2
    counts.append(total)
    return sorted(set(counts))


def noise_floor_curve(
    frames: list[DepthFrame],
    roi: Roi,
    params: AnalyzeParams | None = None,
    *,
    frame_counts: list[int] | None = None,
    k: float = 3.0,
) -> list[CurvePoint]:
    """对录制序列做前缀切片，得到噪声底与可分辨高度随帧数的变化。

    用**前缀**而不是随机窗口，是为了让曲线可复现且单调可比：N 帧的结果始终是
    同一个静止场景的前 N 帧，不同 N 之间只有累积量在变。

    退化输入（分析返回 ``warnings``）会以 ``ok=False`` 与 ``NaN`` 出现在曲线上，
    而不是中断整个扫描——扫描本身要能容忍个别帧数测不出来。
    """
    params = params or AnalyzeParams()
    if not frames:
        return []
    counts = frame_counts or default_frame_counts(len(frames))

    points: list[CurvePoint] = []
    for count in counts:
        if count <= 0 or count > len(frames):
            continue
        result = analyze(frames[:count], roi, params)
        ok = result.ok
        noise_floor = result.metrics.noise_floor_mm
        points.append(
            CurvePoint(
                frame_count=count,
                noise_floor_mm=noise_floor,
                min_resolvable_mm=(k * noise_floor if np.isfinite(noise_floor) else float("nan")),
                peak_height_mm=result.metrics.peak_height_mm,
                valid_ratio=result.metrics.valid_ratio,
                ok=ok,
            )
        )
    return points


def resolve_limit(
    spec: SyntheticSpec,
    heights_mm: list[float],
    *,
    frame_count: int = 64,
    roi: Roi | None = None,
    params: AnalyzeParams | None = None,
    tolerance_mm: float = 0.05,
) -> list[tuple[float, float, bool]]:
    """注入已知高度的凸起，找出能被准确恢复的最小高度。

    返回 ``(真实高度, 恢复高度, 是否在容差内)`` 的列表。这是**经验**的可分辨
    极限，比 ``k × 噪声底`` 那个解析估计更可信，因为解析估计忽略了稳健基准面
    拟合与飞点屏蔽引入的额外损耗。

    每个高度使用完全相同的噪声序列（``spec.seed`` 固定），因此结论不受噪声
    抽样差异影响——否则测出的"最小可分辨高度"会混入随机性。
    """
    from dataclasses import replace

    results: list[tuple[float, float, bool]] = []
    for height in heights_mm:
        case = replace(spec, bump_height_mm=height)
        roi_case = roi or Roi(0, 0, spec.width, spec.height)
        stack = [
            DepthFrame(depth_mm=frame, index=index)
            for index, frame in enumerate(synthetic_frames(case, frame_count))
        ]
        result = analyze(stack, roi_case, params)
        recovered = result.metrics.peak_height_mm
        truth = float(ground_truth_height_mm(case).max())
        within = bool(np.isfinite(recovered) and abs(recovered - truth) <= tolerance_mm)
        results.append((truth, recovered, within))
    return results
