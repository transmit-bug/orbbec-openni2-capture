"""wheal —— 过敏原风团 2.5D 高度图分析。

属于 issue #1 的可行性调研。**这不是产品，是 spike**：目标是产出可复现的
量化结论（噪声底、可分辨的最小隆起高度、失效条件），结论有可能是"不适用"。

两个接缝：

- :func:`analyze` —— Seam 1，纯函数，不接触硬件。
- :class:`FrameSource` / :class:`PgmSequence` —— Seam 2，唯一允许接触磁盘的地方。

符号约定：``height_residual = baseline - depth``，正值表示隆起。
"""

from .accumulate import AccumulateParams, FusedDepth, accumulate
from .analyze import analyze, analyze_source
from .baseline import BaselineFit, fit_baseline
from .frames import (
    DepthFrame,
    FrameSource,
    PgmSequence,
    Roi,
    SequenceMetadata,
    valid_mask,
    write_sequence,
)
from .geometry import (
    Mesh2p5D,
    build_mesh,
    depth_to_points,
    orient_toward_viewer,
    render_overview,
    vertex_normals,
    write_ply,
)
from .intrinsics import Intrinsics
from .metrics import (
    AnalysisResult,
    AnalyzeParams,
    WhealMetrics,
    min_resolvable_height_mm,
)
from .pgm import read_pgm, write_pgm
from .preview import depth_to_rgb, render_depth_preview, render_sequence_preview, robust_range
from .sweep import CurvePoint, default_frame_counts, noise_floor_curve, resolve_limit
from .synthetic import SyntheticSpec, ground_truth_height_mm
from .synthetic import frames as synthetic_frames

__version__ = "0.1.0"

__all__ = [
    "AccumulateParams",
    "AnalysisResult",
    "AnalyzeParams",
    "BaselineFit",
    "CurvePoint",
    "DepthFrame",
    "FrameSource",
    "FusedDepth",
    "Intrinsics",
    "Mesh2p5D",
    "PgmSequence",
    "Roi",
    "SequenceMetadata",
    "SyntheticSpec",
    "WhealMetrics",
    "__version__",
    "accumulate",
    "analyze",
    "analyze_source",
    "build_mesh",
    "default_frame_counts",
    "depth_to_points",
    "depth_to_rgb",
    "fit_baseline",
    "ground_truth_height_mm",
    "min_resolvable_height_mm",
    "noise_floor_curve",
    "orient_toward_viewer",
    "read_pgm",
    "render_depth_preview",
    "render_overview",
    "render_sequence_preview",
    "resolve_limit",
    "robust_range",
    "synthetic_frames",
    "valid_mask",
    "vertex_normals",
    "write_pgm",
    "write_ply",
    "write_sequence",
]
