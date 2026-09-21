"""相机内参与反投影。

**本模块的所有数值都是未标定的默认值。** 调研阶段先不标定，但必须把这件事
写在代码里而不是靠记忆——未标定的横向尺度会以系统性形变的形式污染模型，
而且从结果上很难看出来。

Astra Pro 640×480 深度的常见出厂值约为 ``fx = fy ≈ 570.3``、``cx ≈ 319.5``、
``cy ≈ 239.5``，对应水平视场角 ``2·atan(320/570.3) ≈ 58.6°``，与 Orbbec
公布的 ~58.4° 基本吻合。个体差异仍存在，正式使用前需要棋盘格标定。

对**纯 Z 向的高度残差**（风团隆起）而言内参不参与计算；但 2.5D 场景建模需要
把像素反投影成点云，横向尺度就进入了关键路径。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: 未标定的 Astra Pro 640×480 默认焦距（像素）。
ASTRA_PRO_FX = 570.3
ASTRA_PRO_CY = 239.5


@dataclass(frozen=True)
class Intrinsics:
    """针孔相机内参。``fx``/``fy`` 单位像素，``cx``/``cy`` 为像素坐标。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    calibrated: bool = False

    @classmethod
    def astra_pro_default(cls, width: int = 640, height: int = 480) -> Intrinsics:
        """未标定的出厂近似值，按分辨率等比缩放。

        ``calibrated=False`` 是刻意的：调用方和报告都应能看出这份内参没有标定过。
        """
        scale = width / 640.0
        return cls(
            fx=ASTRA_PRO_FX * scale,
            fy=ASTRA_PRO_FX * scale,
            cx=(width - 1) / 2.0,
            cy=ASTRA_PRO_CY * (height / 480.0),
            width=width,
            height=height,
            calibrated=False,
        )

    @classmethod
    def from_hfov(
        cls, hfov_deg: float, width: int, height: int, *, calibrated: bool = False
    ) -> Intrinsics:
        """从水平视场角反推焦距，供驱动能报出 FOV 时使用。"""
        if not 0.0 < hfov_deg < 180.0:
            raise ValueError(f"水平视场角非法: {hfov_deg}")
        fx = (width / 2.0) / np.tan(np.deg2rad(hfov_deg) / 2.0)
        return cls(
            fx=fx,
            fy=fx,
            cx=(width - 1) / 2.0,
            cy=(height - 1) / 2.0,
            width=width,
            height=height,
            calibrated=calibrated,
        )

    @property
    def hfov_deg(self) -> float:
        return float(2.0 * np.rad2deg(np.arctan((self.width / 2.0) / self.fx)))

    @property
    def vfov_deg(self) -> float:
        return float(2.0 * np.rad2deg(np.arctan((self.height / 2.0) / self.fy)))

    def pixel_grid(self) -> tuple[np.ndarray, np.ndarray]:
        u = np.arange(self.width, dtype=np.float64)[None, :]
        v = np.arange(self.height, dtype=np.float64)[:, None]
        return np.broadcast_to(u, (self.height, self.width)), np.broadcast_to(
            v, (self.height, self.width)
        )

    def unproject(self, depth_mm: np.ndarray) -> np.ndarray:
        """把深度图反投影成点云。

        输入 ``(H, W)`` 毫米；返回 ``(H, W, 3)``，**单位为米**（3D 查看器的通行约定）。
        """
        if depth_mm.shape != (self.height, self.width):
            raise ValueError(f"深度图尺寸 {depth_mm.shape} 与内参 {(self.height, self.width)} 不符")
        z = np.asarray(depth_mm, dtype=np.float64) / 1000.0
        u, v = self.pixel_grid()
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.stack([x, y, z], axis=-1)

    def lateral_mm_per_px(self, z_mm: float) -> float:
        """深度 ``z_mm`` 处一个像素对应的横向尺寸（毫米）。

        判断特征是否看得见时用得上：未标定内参的横向误差是**系统性**的，
        不随累积帧数减小。
        """
        return float(z_mm) / self.fx

    def description(self) -> str:
        source = "已标定" if self.calibrated else "**未标定**（出厂近似值）"
        return (
            f"{self.width}x{self.height} fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f} "
            f"HFOV={self.hfov_deg:.1f}° VFOV={self.vfov_deg:.1f}° — {source}"
        )
