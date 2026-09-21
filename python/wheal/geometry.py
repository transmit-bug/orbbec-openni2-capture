"""2.5D 几何重建 —— 从一段静态深度序列生成单视角表面模型。

**本模块不认识风团，也不认识任何具体场景。** 输入是累积后的深度图，输出是点云、
网格与文件；里面没有一处分支在问“前面是什么东西”。判断要不要补一个基准面不是
几何问题，是**测量模型**问题：

- :mod:`wheal.geometry`（本模块）—— 深度值本身就是模型。没有可以拟合掉的基准面；
- :mod:`wheal.analyze` —— 有基准面（皮肤），报的是相对基准面的 Z 残差。

两者不是并列的两条流水线，而是**包含关系**：``analyze`` = 本模块的累积 + 一个
基准面。对着平墙跑 ``analyze``，``fit_baseline`` 会老老实实把墙拟合成基准面，
报出墙的粗糙度——它不需要知道那是墙。

本模块只做四件事：

1. **反投影** —— 用内参把深度图变成点云；
2. **三角化** —— 规则网格上每格两个三角形，并在深度不连续处**切断**，
   避免把前景和背景用一片薄面缝在一起（那会在轮廓上拉出一层虚假的“蛛网”）；
3. **定向** —— 由面法向平均出顶点法向，并翻到指向观察者的一侧；
4. **导出** —— PLY（含顶点色与法向）与总览图。

累积不在这里，在 :mod:`wheal.accumulate`——两个消费者共用同一份累积，
见那里的说明。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .accumulate import FusedDepth
from .intrinsics import Intrinsics

#: 一个网格单元内深度极差超过该值即不连面（毫米）。轮廓处前景/背景相差远大于此。
DEFAULT_DISCONTINUITY_MM = 25.0


@dataclass(frozen=True)
class Mesh2p5D:
    """单视角 2.5D 表面网格。顶点单位为**米**。"""

    vertices: np.ndarray  # (M, 3) float64
    faces: np.ndarray  # (F, 3) int32
    vertex_index: np.ndarray  # (H, W) int32, -1 表示该像素无顶点

    @property
    def vertex_count(self) -> int:
        return int(self.vertices.shape[0])

    @property
    def face_count(self) -> int:
        return int(self.faces.shape[0])

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    def extent_m(self) -> np.ndarray:
        low, high = self.bounds()
        return high - low


def depth_to_points(depth_mm: np.ndarray, valid: np.ndarray, intrinsics: Intrinsics) -> np.ndarray:
    """反投影成点云 ``(H, W, 3)``，单位米。无效处为 ``NaN``。"""
    points = intrinsics.unproject(depth_mm)
    return np.where(valid[..., None], points, np.nan)


def build_mesh(
    depth_mm: np.ndarray,
    valid: np.ndarray,
    intrinsics: Intrinsics,
    *,
    discontinuity_mm: float = DEFAULT_DISCONTINUITY_MM,
) -> Mesh2p5D:
    """在规则网格上三角化，并在深度不连续处切断。

    切断是必须的：深度图的轮廓处，相邻两像素可能相差数十毫米（前后景）。
    若照常连面，会在物体轮廓四周拉出一层斜置的虚假表面，从某些角度看像是
    物体"长出了裙边"。这个问题在点云里不存在，只在网格化时出现。
    """
    height, width = depth_mm.shape
    vertex_index = np.full((height, width), -1, dtype=np.int32)
    vertex_index[valid] = np.arange(int(valid.sum()), dtype=np.int32)
    points = depth_to_points(depth_mm, valid, intrinsics)
    vertices = points[valid]

    # 四角都有效，且格内深度极差小于门限，才连面。
    quad_ok = valid[:-1, :-1] & valid[:-1, 1:] & valid[1:, 1:] & valid[1:, :-1]
    corners = (
        depth_mm[:-1, :-1],
        depth_mm[:-1, 1:],
        depth_mm[1:, 1:],
        depth_mm[1:, :-1],
    )
    stacked = np.stack(corners)
    # 用 fmax/fmin 而不是 nanmax/nanmin：前者忽略 NaN 且不产生 All-NaN 警告，
    # 四角全无效的格子本来就会被 quad_ok 排除。
    spread = np.fmax.reduce(stacked, axis=0) - np.fmin.reduce(stacked, axis=0)
    quad_ok &= np.isfinite(spread) & (spread < discontinuity_mm)

    rows, cols = np.nonzero(quad_ok)
    if rows.size == 0:
        return Mesh2p5D(vertices, np.zeros((0, 3), dtype=np.int32), vertex_index)

    i00 = vertex_index[:-1, :-1][rows, cols]
    i01 = vertex_index[:-1, 1:][rows, cols]
    i11 = vertex_index[1:, 1:][rows, cols]
    i10 = vertex_index[1:, :-1][rows, cols]
    faces = np.concatenate(
        [
            np.stack([i00, i01, i11], axis=1),
            np.stack([i00, i11, i10], axis=1),
        ]
    ).astype(np.int32)
    return Mesh2p5D(vertices, faces, vertex_index)


def vertex_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """由面法向面积加权平均出顶点法向，返回单位向量。

    没有法向的 PLY 在任何带光照的渲染器里都是一团黑——渲染器只能用默认材质，
    而默认材质往往不带自发辐射。导出法向不是锦上添花，是"能不能看见"的前提。
    """
    vertices = np.asarray(vertices, dtype=np.float64)
    normals = np.zeros_like(vertices)
    if faces.size:
        faces = np.asarray(faces, dtype=np.int64)
        tri = vertices[faces]
        # 叉积的模等于两倍三角形面积，所以直接累加就自带面积加权。
        weighted = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        for corner in range(3):
            np.add.at(normals, faces[:, corner], weighted)

    length = np.linalg.norm(normals, axis=1, keepdims=True)
    # 孤立顶点（没有任何面引用）没有面法向可平均；给一个占位方向而不是 NaN，
    # 因为 NaN 法向会让整块网格在渲染器里消失得无影无踪。
    degenerate = (length[:, 0] <= 0)[:, None]
    normals = np.where(degenerate, np.array([0.0, 0.0, -1.0]), normals)
    length = np.linalg.norm(normals, axis=1, keepdims=True)
    return (normals / np.where(length > 0, length, 1.0)).astype(np.float64)


def orient_toward_viewer(
    vertices: np.ndarray, normals: np.ndarray, viewer: np.ndarray | None = None
) -> np.ndarray:
    """把法向翻到指向观察者的一侧。

    这台相机的单视角 2.5D 面法向朝 ``+z``（远离相机），而相机在原点、表面在
    ``z ≈ +0.46 m``——也就是说从相机方向看到的是**背面**。背面剔除之后什么都
    看不见，反向光照之后是一片黑。两种表现都被当成"文件坏了"，其实只是朝向。
    """
    viewer = np.zeros(3) if viewer is None else np.asarray(viewer, dtype=np.float64)
    to_viewer = viewer - np.asarray(vertices, dtype=np.float64)
    facing_away = np.einsum("ij,ij->i", normals, to_viewer) < 0.0
    return np.where(facing_away[:, None], -normals, normals)


def write_ply(
    path: str | Path,
    vertices: np.ndarray,
    *,
    faces: np.ndarray | None = None,
    colors: np.ndarray | None = None,
    normals: np.ndarray | None = None,
    fmt: str = "binary",
) -> Path:
    """写 PLY。顶点单位**米**。

    ``colors`` / ``normals`` 都是可选的，但**强烈建议给上**：只有顶点坐标的 PLY
    在 MeshLab / Blender / CloudCompare 里打开是黑的（没有材质、没有光照信息），
    而那是查看器的问题还是文件的问题，从屏幕上分不出来。

    ``fmt`` 选 ``"binary"``（默认）或 ``"ascii"``。

    为什么二进制是默认却还必须留着 ASCII：**Blender 5.2.2 的二进制 PLY 读取器
    是错的**。实测（每一项都独立复现过，见 tests/test_geometry.py::TestPlyReaderInterop）：

    =======================  ============================
    vertex 元素里的属性         Blender 5.2.2 读到的位置
    =======================  ============================
    ``x y z``                正确（唯一的正确情形）
    ``x y z foo``             ``y z foo``——整体错位一个属性
    ``x y z nx ny nz``        读到法向（单位为1，于是包围盒变成 ±1）
    ``x y z red..blue``       垃圾 float（量级 1e38）
    ``x y z nx ny nz rgb``    垃圾 float
    =======================  ============================

    它不是不认颜色，是**整个二进制路径的位置解析就错了**，碰巧只在“顶点元素恰好
    只有 3 个 float 属性”时看不见。ASCII 路径完全正常，所以选 ASCII 就能让
    Blender 打开。二进制保留为默认是因为它小、且是我们自己逐字节验证过的存档格式。
    """
    vertices = np.asarray(vertices, dtype=np.float32)
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError(f"顶点必须是 (M,3)，收到 {vertices.shape}")
    if fmt not in {"binary", "ascii"}:
        raise ValueError(f"fmt 必须是 'binary' 或 'ascii'，收到 {fmt!r}")
    if faces is None:
        faces = np.zeros((0, 3), dtype=np.int32)
    faces = np.asarray(faces, dtype=np.int32)
    if faces.size and (faces.ndim != 2 or faces.shape[1] != 3):
        raise ValueError(f"面必须是 (F,3)，收到 {faces.shape}")

    has_color = colors is not None
    if has_color:
        colors = np.asarray(colors, dtype=np.uint8)
        if colors.shape[0] != vertices.shape[0]:
            raise ValueError("颜色数量与顶点数量不符")
        if colors.ndim != 2 or colors.shape[1] != 3:
            raise ValueError(f"颜色必须是 (M,3)，收到 {colors.shape}")

    has_normals = normals is not None
    if has_normals:
        normals = np.asarray(normals, dtype=np.float32)
        if normals.shape != vertices.shape:
            raise ValueError(f"法向必须与顶点同形，收到 {normals.shape}")

    header = [
        "ply",
        "format ascii 1.0" if fmt == "ascii" else "format binary_little_endian 1.0",
        "comment units: meters",
        "comment generated by wheal (uncalibrated intrinsics unless stated)",
        f"element vertex {vertices.shape[0]}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if has_normals:
        # 顺序必须与写入顺序一致：PLY 是靠属性出现顺序逐字节切分的，
        # header 与数据块一旦错位，查看器读到的就是垃圾（而且不会报错）。
        header += [
            "property float nx",
            "property float ny",
            "property float nz",
        ]
    if has_color:
        header += [
            "property uchar red",
            "property uchar green",
            "property uchar blue",
        ]
    header += [
        f"element face {faces.shape[0]}",
        "property list uchar int vertex_indices",
        "end_header",
    ]

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    if fmt == "ascii":
        # 6 位小数：顶点单位是米，所以这是 1 微米分辨率，远细于传感器的毫米级精度。
        with target.open("w", encoding="ascii") as handle:
            handle.write("\n".join(header) + "\n")
            for index, vertex in enumerate(vertices):
                fields = [f"{value:.6f}" for value in vertex]
                if has_normals:
                    fields += [f"{value:.6f}" for value in normals[index]]
                if has_color:
                    fields += [str(int(value)) for value in colors[index]]
                handle.write(" ".join(fields) + "\n")
            for face in faces:
                handle.write(f"3 {int(face[0])} {int(face[1])} {int(face[2])}\n")
        return target

    with target.open("wb") as handle:
        handle.write(("\n".join(header) + "\n").encode("ascii"))
        handle.write(vertices.astype("<f4").tobytes())
        if has_normals:
            handle.write(normals.astype("<f4").tobytes())
        if has_color:
            handle.write(colors.tobytes())
        if faces.shape[0]:
            # 必须用结构化 dtype 逐记录打包。写成 np.hstack([counts, faces]) 会
            # 把 uint8 计数提升到 int32，每张面变成 4 个 int32 = 16 字节，
            # 整个面块错位——文件能写出、大小看着也合理，但任何查看器都读不开。
            records = np.empty(faces.shape[0], dtype=np.dtype([("n", "u1"), ("v", "<i4", (3,))]))
            records["n"] = 3
            records["v"] = faces.astype("<i4")
            handle.write(records.tobytes())
    return target


def shaded_relief(points: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """由点云法向做朗伯着色，得到可读的 2.5D 浮雕图。

    比 3D 散点图有用得多：散点图把遮挡与透视混在一起，换个视角就什么都看不出来，
    而浮雕图直接把“表面长什么样”画成一张图。
    """
    filled = np.where(valid[..., None], points, np.nan)
    dx = np.gradient(np.nan_to_num(filled, nan=0.0), axis=1)
    dy = np.gradient(np.nan_to_num(filled, nan=0.0), axis=0)
    normal = np.cross(dy, dx)
    norm = np.linalg.norm(normal, axis=-1, keepdims=True)
    with np.errstate(invalid="ignore", divide="ignore"):
        normal = normal / norm
    light = np.array([-0.4, -0.6, 0.7])
    light = light / np.linalg.norm(light)
    shade = np.abs(np.nansum(normal * light, axis=-1))
    return np.where(valid, shade, np.nan)


def render_overview(
    fused: FusedDepth,
    intrinsics: Intrinsics,
    path: str | Path,
    *,
    point_stride: int = 4,
) -> Path:
    """四联图：深度图 / 浮雕 / 覆盖掩码 / 逐行剖面。

    “逐行剖面”那一格是刻意留的：这台相机的深度误差主要沿**行**方向成带出现
    （同一行内相邻列差中位仅 0.2 mm，相邻行差 p90 却达 30 mm）。不看这个剖面，
    很容易把行相关伪影当成真实几何。
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    height, width = fused.depth_mm.shape
    points = depth_to_points(fused.depth_mm, fused.valid, intrinsics)

    figure = plt.figure(figsize=(13.5, 9.2))

    ax = figure.add_subplot(2, 2, 1)
    shown = np.where(np.isfinite(fused.depth_mm), fused.depth_mm, np.nan)
    image = ax.imshow(shown, cmap="turbo", interpolation="nearest")
    ax.set_title(f"累积深度图（{fused.frame_count} 帧）—— 黑 = 无数据")
    figure.colorbar(image, ax=ax, fraction=0.046, label="mm")

    ax = figure.add_subplot(2, 2, 2)
    relief = shaded_relief(points, fused.valid)
    ax.imshow(relief, cmap="gray", interpolation="nearest", vmin=0.0, vmax=1.0)
    ax.set_title("阴影浮雕（法向朗伯着色）\n亮/暗 = 表面朝向")

    ax = figure.add_subplot(2, 2, 3)
    ax.imshow(fused.valid, cmap="gray", interpolation="nearest")
    ax.set_title(
        f"有效像素掩码\n单帧 {fused.per_frame_coverage * 100:.0f}% → "
        f"累积 {fused.fused_coverage * 100:.0f}%（填洞 {fused.hole_fill_gain:.2f}x）"
    )

    ax = figure.add_subplot(2, 2, 4)
    row_median = np.array(
        [np.nanmedian(fused.depth_mm[r]) if fused.valid[r].any() else np.nan for r in range(height)]
    )
    ax.plot(row_median, np.arange(height), ".-", markersize=2, linewidth=0.6)
    ax.invert_yaxis()
    ax.set_xlabel("逐行中位深度 (mm)")
    ax.set_ylabel("行号")
    ax.set_title("逐行剖面 —— 行相关伪影诊断\n横向毛刺 = 行间系统偏差（非真实几何）")
    ax.grid(alpha=0.3)

    figure.tight_layout()
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(target, dpi=105)
    plt.close(figure)
    return target
