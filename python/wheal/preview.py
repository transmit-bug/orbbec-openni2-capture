"""把"看不见的"中间产物变成能看的图。

这个模块存在的理由是两个**会静默发生**的陷阱：文件写出来了、大小看着也正常、
没有任何报错，但打开是一团黑。

1. **PGM 深度图天生是黑的。** C++ 侧写出的是 16-bit 大端 PGM，深度值以毫米直接
   存储（这台相机上是 460..630）。任何看图程序要么按 8-bit 读——那就只剩高字节，
   460 mm 变成 ``460 >> 8 == 1``，整幅图落在 0..2/255——要么按 16-bit 读但仍把
   0..65535 当成显示范围，于是同样全黑。深度图**必须重新归一化到有效区间**才能看，
   而"归一化"这一步恰恰是通用查看器不会替你做、也不会提醒你的。

2. **PLY 在带光照的渲染器里天生是黑的。** 导出的点云/网格既没有顶点色也没有法向，
   渲染器就拿默认材质画它；而单视角 2.5D 网格的面法向朝**远离相机**的一侧
   （相机在原点、表面在 z≈+0.46 m），于是从相机方向看到的是背面，被背面剔除或
   被反向光照，结果还是黑。见 :func:`vertex_normals` 与
   :func:`orient_toward_viewer`。

两件事的共同点：**失败的表现是"黑"，而黑同时也是合法的数据**（无返回的像素就是
无数据）。所以这里全部按显式约定处理——空洞一律涂成纯黑并标注出来，而不是让它
们混进色标里冒充数值。
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # 必须在 pyplot 之前；预览生成不应要求图形环境

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .fonts import t as _t  # noqa: E402

#: 空洞的显示颜色。选纯黑是因为深度图里"黑 = 无数据"是这套流程的既有约定。
HOLE_RGB = (0, 0, 0)


def default_valid_mask(depth_mm: np.ndarray) -> np.ndarray:
    """传感器约定的有效性：``0`` / ``65535`` 是没返回数据，非有限值是 NaN 污染。

    与 :func:`wheal.frames.valid_mask` 的区别：那个还会叠一个信任距离上限
    （``max_valid_mm``），是**分析**用的；这里只要能在屏幕上画出来，是**显示**用的。
    """
    depth = np.asarray(depth_mm)
    return (depth != 0) & (depth != 65535) & np.isfinite(depth)


def robust_range(
    values: np.ndarray, *, low: float = 0.5, high: float = 99.5
) -> tuple[float, float]:
    """按分位数取显示区间。

    不用 min/max：深度图里总有几个飞点（p99 偏差可达 std 的 4 倍以上），
    照着极值拉伸会把整幅图压成一条窄带，等于又变回一张黑图。
    """
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return 0.0, 1.0
    lo, hi = (float(v) for v in np.percentile(finite, [low, high]))
    if hi - lo < 1e-9:
        # 完全平坦的输入：给一个人造窗口，否则除以零后整幅图变成常数色。
        return lo - 0.5, lo + 0.5
    return lo, hi


def depth_to_rgb(
    depth_mm: np.ndarray,
    valid: np.ndarray | None = None,
    *,
    vmin: float | None = None,
    vmax: float | None = None,
    cmap: str = "turbo",
) -> np.ndarray:
    """深度图 → ``(H, W, 3)`` uint8 RGB，空洞为纯黑。

    ``vmin``/``vmax`` 省略时用有效值的稳健分位区间。**注意全零输入**（整帧无数据）
    会得到全黑——那是正确的，不是 bug。
    """
    depth = np.asarray(depth_mm, dtype=np.float64)
    mask = np.isfinite(depth) if valid is None else (np.asarray(valid) & np.isfinite(depth))
    if vmin is None or vmax is None:
        auto_lo, auto_hi = robust_range(depth[mask])
        vmin = auto_lo if vmin is None else vmin
        vmax = auto_hi if vmax is None else vmax

    normalized = np.clip((np.where(mask, depth, vmin) - vmin) / (vmax - vmin), 0.0, 1.0)
    rgb = (matplotlib.colormaps[cmap](normalized)[..., :3] * 255.0).astype(np.uint8)
    rgb[~mask] = HOLE_RGB
    return rgb


def render_depth_preview(
    depth_mm: np.ndarray,
    path: str | Path,
    *,
    valid: np.ndarray | None = None,
    title: str | None = None,
    cmap: str = "turbo",
) -> Path:
    """单幅深度图 → 带色标与覆盖率的 PNG。这是"能直接打开看"的那种图。"""
    depth = np.asarray(depth_mm, dtype=np.float64)
    mask = default_valid_mask(depth) if valid is None else np.asarray(valid)
    lo, hi = robust_range(depth[mask])

    figure, axes = plt.subplots(figsize=(7.6, 6.0))
    shown = np.where(mask, depth, np.nan)
    image = axes.imshow(shown, cmap=cmap, vmin=lo, vmax=hi, interpolation="nearest")
    axes.set_facecolor("black")
    axes.set_title(title or _t("深度图（黑 = 无数据）", "depth map (black = no data)"))
    figure.colorbar(image, ax=axes, fraction=0.046, label="mm")
    axes.text(
        0.02,
        0.02,
        _t(
            f"有效 {mask.mean() * 100:.1f}%   显示区间 {lo:.0f}..{hi:.0f} mm",
            f"valid {mask.mean() * 100:.1f}%   display range {lo:.0f}..{hi:.0f} mm",
        ),
        transform=axes.transAxes,
        color="white",
        fontsize=9,
        bbox={"facecolor": "black", "alpha": 0.6, "pad": 3},
    )

    figure.tight_layout()
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(target, dpi=110)
    plt.close(figure)
    return target


def render_sequence_preview(
    depths: list[np.ndarray],
    path: str | Path,
    *,
    columns: int = 6,
    title: str | None = None,
    cmap: str = "turbo",
) -> Path:
    """一段序列 → 缩略图网格，用于一眼确认录制内容与空洞模式。

    刻意让所有小图共用同一个显示区间：如果每张各自归一化，覆盖率的差异会被
    归一化抹平，看图的结论就会和统计数字相反。
    """
    if not depths:
        raise ValueError("depths 为空")

    masks = [default_valid_mask(d) for d in depths]
    pooled = np.concatenate([d[m] for d, m in zip(depths, masks, strict=True) if m.any()])
    lo, hi = robust_range(pooled)

    rows = (len(depths) + columns - 1) // columns
    figure, axes = plt.subplots(rows, columns, figsize=(2.1 * columns, 1.9 * rows), squeeze=False)
    for index, (depth, mask) in enumerate(zip(depths, masks, strict=True)):
        axis = axes[index // columns][index % columns]
        axis.imshow(np.where(mask, depth, np.nan), cmap=cmap, vmin=lo, vmax=hi)
        axis.set_facecolor("black")
        axis.set_title(f"#{index}  {mask.mean() * 100:.0f}%", fontsize=8)
        axis.set_xticks([])
        axis.set_yticks([])
    for index in range(len(depths), rows * columns):
        axes[index // columns][index % columns].axis("off")

    figure.suptitle(
        title
        or _t(
            f"序列缩略图（{len(depths)} 帧，共同显示区间 {lo:.0f}..{hi:.0f} mm）",
            f"sequence thumbnails ({len(depths)} frames, shared range {lo:.0f}..{hi:.0f} mm)",
        )
    )
    figure.tight_layout()
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(target, dpi=105)
    plt.close(figure)
    return target
