"""合成深度序列。

调研需要一条**已知答案**的基准线：给定一个确定高度的凸起和确定的噪声水平，
算法能不能把它恢复出来、恢复得准不准。没有这条基准线，真实数据的结论就无法
判断是"传感器不行"还是"算法写错了"。

这个模块同时服务于两个用途：

- 测试：断言已知凸起被恢复；
- 报告的自我校验：在真实数据之外，附上一条合成曲线作为方法有效性的证据。

``frames()`` 对固定 ``seed`` 是**前缀稳定**的——``frames(spec, 10)[:5]`` 与
``frames(spec, 5)`` 逐字节相同。扫帧数曲线依赖这个性质，否则不同帧数之间
不可比。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

#: 空洞掩码用独立的固定种子，使所有帧共享同一组空洞（静止场景的遮挡是静止的）。
_HOLE_SEED_OFFSET = 12345


@dataclass(frozen=True)
class SyntheticSpec:
    """合成场景的几何与噪声参数。全部单位毫米，坐标以像素计。"""

    shape: tuple[int, int] = (120, 160)
    baseline_mm: float = 600.0
    tilt_x_mm: float = 0.0
    tilt_y_mm: float = 0.0
    curvature_mm: float = 0.0
    bump_height_mm: float = 0.0
    bump_sigma_px: float = 8.0
    bump_center: tuple[float, float] | None = None
    step_height_mm: float = 0.0
    step_axis: Literal["u", "v"] = "u"
    step_position_px: int | None = None
    noise_sigma_mm: float = 0.0
    hole_fraction: float = 0.0
    hole_mode: Literal["static", "dynamic"] = "static"
    seed: int = 0

    @property
    def height(self) -> int:
        return self.shape[0]

    @property
    def width(self) -> int:
        return self.shape[1]


def _normalized_grid(shape: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    height, width = shape
    u = np.linspace(-1.0, 1.0, width)
    v = np.linspace(-1.0, 1.0, height)
    return np.meshgrid(u, v)


def bump_profile(spec: SyntheticSpec) -> np.ndarray:
    """真实的隆起高度场（即风团本身），单位 mm。

    这是 ``analyze`` 应当恢复出来的目标，也是所有精度断言的参照。
    """
    if spec.bump_height_mm == 0.0:
        return np.zeros(spec.shape, dtype=np.float64)
    u, v = _normalized_grid(spec.shape)
    center_u, center_v = spec.bump_center or (0.0, 0.0)
    # 中心以像素指定，转成归一化坐标后与 u/v 网格一致。
    span_u = max(spec.width - 1, 1) / 2.0
    span_v = max(spec.height - 1, 1) / 2.0
    du = (u - center_u / span_u) * span_u
    dv = (v - center_v / span_v) * span_v
    return spec.bump_height_mm * np.exp(-0.5 * (du**2 + dv**2) / (spec.bump_sigma_px**2))


def surface_mm(spec: SyntheticSpec) -> np.ndarray:
    """含基准面与隆起的完整深度场（无噪声），单位 mm。

    隆起使深度**变小**（朝相机凸起），与 ``analyze`` 的符号约定一致。
    """
    u, v = _normalized_grid(spec.shape)
    surface = np.full(spec.shape, spec.baseline_mm, dtype=np.float64)
    surface += spec.tilt_x_mm * u + spec.tilt_y_mm * v
    surface += spec.curvature_mm * (u**2 + v**2)

    if spec.step_height_mm != 0.0:
        position = spec.step_position_px
        if position is None:
            position = (spec.width if spec.step_axis == "u" else spec.height) // 2
        index = np.arange(spec.width if spec.step_axis == "u" else spec.height)
        mask = (index > position).astype(np.float64)
        surface -= spec.step_height_mm * (mask[None, :] if spec.step_axis == "u" else mask[:, None])

    return surface - bump_profile(spec)


def _hole_mask(spec: SyntheticSpec) -> np.ndarray:
    """静止的空间空洞掩码（模拟遮挡）。``dynamic`` 模式不使用它。"""
    if spec.hole_fraction <= 0.0:
        return np.zeros(spec.shape, dtype=bool)
    rng = np.random.default_rng(spec.seed + _HOLE_SEED_OFFSET)
    return rng.random(spec.shape) < spec.hole_fraction


def frames(spec: SyntheticSpec, count: int) -> list[np.ndarray]:
    """生成 ``count`` 帧带噪声的 ``uint16`` 深度图。

    逐帧独立加噪。空洞有两种模式，它们对应**不同**的物理现象，不能混为一谈：

    - ``static``：遮挡造成的洞（桌子后、物体后）。时间累积**填不了**。
    - ``dynamic``：结构光匹配失败造成的洞。**逐帧重掷**，时间累积能填。

    真实 Astra Pro 的主要是后者：实测同一场景相邻两帧的空洞掩码 IoU 仅 0.497
    （接近独立），所以 120 帧累积把覆盖率从 43% 提到了 94%。合成数据默认用
    ``static``（更难的情形）；需要验证填洞能力时显式选 ``dynamic``。

    ``dynamic`` 模式下也是前缀稳定的：噪声与掩码按固定顺序从同一个 rng 抽取。
    """
    if count <= 0:
        raise ValueError(f"count 必须为正: {count}")
    rng = np.random.default_rng(spec.seed)
    surface = surface_mm(spec)
    holes = _hole_mask(spec)
    dynamic = spec.hole_mode == "dynamic" and spec.hole_fraction > 0.0

    out: list[np.ndarray] = []
    for _ in range(count):
        noisy = surface + rng.normal(0.0, spec.noise_sigma_mm, spec.shape)
        frame = np.clip(np.rint(noisy), 1, 65534).astype(np.uint16)
        if dynamic:
            mask = rng.random(spec.shape) < spec.hole_fraction
            frame = np.where(mask, np.uint16(0), frame)
        elif holes.any():
            frame = np.where(holes, np.uint16(0), frame)
        out.append(frame)
    return out


def ground_truth_height_mm(spec: SyntheticSpec) -> np.ndarray:
    """真实的隆起高度场，供精度断言使用。"""
    return bump_profile(spec)
