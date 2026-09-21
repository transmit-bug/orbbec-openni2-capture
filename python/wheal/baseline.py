"""稳健基准面拟合。

这是整个方法成立的关键一步。风团的隆起量被定义为**相对基准面的高度残差**，
因此基准面本身必须不被风团拽偏。

用普通最小二乘会有一个隐蔽且致命的后果：风团会把曲面朝自己方向拉，
于是测出的隆起量**系统性偏小**，而且看不出错——因为残差图看起来很平滑。
所以这里用 IRLS（迭代重加权最小二乘）+ Tukey biweight，把风团当作离群值剔除。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .stats import MAD_TO_SIGMA

#: 拟合所需的最少像素数，取设计矩阵列数的该倍数。低于此值则基准面无意义。
MIN_PIXELS_PER_TERM = 8


def design_terms(degree: int) -> int:
    """阶数为 ``degree`` 的二维多项式项数。"""
    if degree < 0:
        raise ValueError(f"基准面阶数不能为负: {degree}")
    return (degree + 1) * (degree + 2) // 2


def _normalized_coords(shape: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """把像素坐标归一化到 [-1, 1]，避免高阶项在数值上病态。"""
    height, width = shape
    u = np.linspace(-1.0, 1.0, width) if width > 1 else np.zeros(width)
    v = np.linspace(-1.0, 1.0, height) if height > 1 else np.zeros(height)
    return np.meshgrid(u, v)


def design_matrix(shape: tuple[int, int], degree: int) -> np.ndarray:
    """构造设计矩阵 ``A``，形状 ``(h*w, n_terms)``，列对应 u^i · v^j（i+j ≤ degree）。"""
    u, v = _normalized_coords(shape)
    columns = []
    for i in range(degree + 1):
        for j in range(degree + 1 - i):
            columns.append((u**i) * (v**j))
    return np.stack([column.ravel() for column in columns], axis=1)


@dataclass(frozen=True)
class BaselineFit:
    """拟合结果。``coefficients`` 与 :func:`design_matrix` 的列顺序一一对应。"""

    coefficients: np.ndarray
    degree: int
    shape: tuple[int, int]
    inlier_mask: np.ndarray
    residual_sigma_mm: float
    iterations: int

    def evaluate(self) -> np.ndarray:
        """在整幅 ROI 网格上求基准面，返回 ``(h, w)``。"""
        return (design_matrix(self.shape, self.degree) @ self.coefficients).reshape(self.shape)


def fit_baseline(
    depth_mm: np.ndarray,
    usable_mask: np.ndarray,
    *,
    degree: int = 2,
    robust_k: float = 3.0,
    max_iterations: int = 8,
) -> BaselineFit | None:
    """IRLS 拟合基准面。``usable_mask`` 之外的数据一律不参与。

    返回 ``None`` 表示有效像素不足以支撑该阶数的拟合——调用方应把它当作
    "本次没测出来"，而不是伪造一个基准面。

    Tukey biweight 的权重在 ``|r| > robust_k·sigma`` 处**恰好归零**，因此风团会被
    完整剔除而不是仅仅降权；这正是我们要的：拟合只用正常皮肤。
    """
    shape = depth_mm.shape
    n_terms = design_terms(degree)
    min_pixels = n_terms * MIN_PIXELS_PER_TERM

    flat_usable = usable_mask.ravel()
    if int(flat_usable.sum()) < min_pixels:
        return None

    matrix = design_matrix(shape, degree)[flat_usable]
    values = depth_mm.ravel()[flat_usable].astype(np.float64)

    weights = np.ones(values.shape[0], dtype=np.float64)
    coefficients = np.zeros(n_terms, dtype=np.float64)
    sigma = float("nan")
    iterations = 0

    for iterations in range(1, max_iterations + 1):  # noqa: B007 - 保留末次迭代号作诊断信息
        root_w = np.sqrt(weights)
        coefficients, *_ = np.linalg.lstsq(matrix * root_w[:, None], values * root_w, rcond=None)
        residuals = values - matrix @ coefficients
        # residuals 已经是残差，直接取绝对值；再减一次中位数会把真实偏差抹掉一块。
        sigma = MAD_TO_SIGMA * float(np.median(np.abs(residuals)))
        if not np.isfinite(sigma) or sigma <= 0.0:
            break
        u = residuals / (robust_k * sigma)
        new_weights = np.where(np.abs(u) < 1.0, (1.0 - u**2) ** 2, 0.0)
        if np.allclose(new_weights, weights, atol=1e-6):
            weights = new_weights
            break
        weights = new_weights

    inlier_mask = np.zeros(flat_usable.shape, dtype=bool)
    if weights.size and np.isfinite(sigma) and sigma > 0.0:
        inlier_mask[flat_usable] = weights > 0.0
    else:
        # 残差全为零（例如完美平面）时 sigma 为 0，所有点都是内点。
        inlier_mask[flat_usable] = True

    # 噪声底只由内点决定，避免风团聚类的离群值把噪声底抬高。
    residuals = values - matrix @ coefficients
    inliers = inlier_mask[flat_usable]
    if inliers.any():
        sigma = MAD_TO_SIGMA * float(np.median(np.abs(residuals[inliers])))
    if not np.isfinite(sigma):
        sigma = 0.0

    return BaselineFit(
        coefficients=coefficients,
        degree=degree,
        shape=shape,
        inlier_mask=inlier_mask.reshape(shape),
        residual_sigma_mm=sigma,
        iterations=iterations,
    )
