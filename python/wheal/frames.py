"""帧来源与帧类型 —— **Seam 2**。

这个模块是整个调研里唯一允许接触外部世界（磁盘）的地方。``analyze`` 只接受
这个模块定义的纯数据类型，因此相机和录像格式的变化都被挡在这里。

两个实现的关系：

- :class:`PgmSequence` —— 从 C++ 录制的目录读。**调研的主力路径**，无相机也能跑。
- 实时相机 —— 有意不在此实现。采集留在 C++（OpenNI2 + 定制驱动 + 旧固件变通），
  通过磁盘交接，避免 pybind11 / GIL / 构建耦合。
"""

from __future__ import annotations

import json
from collections.abc import Iterator, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

import numpy as np

from .pgm import read_pgm

#: 传感器约定：0 表示无返回，65535 表示饱和/无效。
INVALID_DEPTH_VALUES: tuple[int, ...] = (0, 65535)

#: C++ 侧写出的文件名格式。
FRAME_GLOB = "frame_*.pgm"
METADATA_FILENAME = "metadata.json"


def valid_mask(depth_mm: np.ndarray, max_valid_mm: float | None = None) -> np.ndarray:
    """深度有效性掩码。

    无效的两个来源要分清：``0`` / ``65535`` 是**传感器没给出数据**（空洞），
    而超过 ``max_valid_mm`` 是**数据存在但不可信**（远距离噪声、飞点）。
    后者默认可关，因为工作距离由 :class:`~wheal.accumulate.AccumulateParams` 决定。
    """
    mask = (depth_mm != 0) & (depth_mm != 65535)
    if max_valid_mm is not None:
        mask &= depth_mm <= max_valid_mm
    return mask


@dataclass(frozen=True)
class DepthFrame:
    """一帧深度图，单位毫米，``uint16``。"""

    depth_mm: np.ndarray
    index: int = 0

    def __post_init__(self) -> None:
        if self.depth_mm.ndim != 2:
            raise ValueError(f"深度图必须是二维，收到 {self.depth_mm.ndim} 维")
        if self.depth_mm.dtype != np.uint16:
            raise ValueError(f"深度图必须是 uint16，收到 {self.depth_mm.dtype}")

    @property
    def height(self) -> int:
        return int(self.depth_mm.shape[0])

    @property
    def width(self) -> int:
        return int(self.depth_mm.shape[1])

    @property
    def shape(self) -> tuple[int, int]:
        return (self.height, self.width)


@dataclass(frozen=True)
class Roi:
    """感兴趣区域，像素坐标。

    调研里 ROI 的作用不是"裁剪出风团"，而是**限定基准面的拟合范围**——
    必须覆盖风团周围的正常皮肤，让基准面有足够约束。
    """

    x: int
    y: int
    w: int
    h: int

    @property
    def slices(self) -> tuple[slice, slice]:
        return (slice(self.y, self.y + self.h), slice(self.x, self.x + self.w))

    def crop(self, array: np.ndarray) -> np.ndarray:
        return array[self.slices]

    def check_within(self, shape: tuple[int, int]) -> None:
        height, width = shape
        if self.w <= 0 or self.h <= 0:
            raise ValueError(f"ROI 尺寸必须为正: {self.w}x{self.h}")
        if self.x < 0 or self.y < 0 or self.x + self.w > width or self.y + self.h > height:
            raise ValueError(f"ROI {self} 超出帧范围 {width}x{height}")

    def as_dict(self) -> dict[str, int]:
        return {"x": self.x, "y": self.y, "w": self.w, "h": self.h}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> Roi:
        return cls(x=int(data["x"]), y=int(data["y"]), w=int(data["w"]), h=int(data["h"]))


@dataclass(frozen=True)
class SequenceMetadata:
    """一次录制的旁路信息。

    ``working_distance_mm`` 是分析结论的自变量之一（"随工作距离如何变化"），
    所以必须随数据一起记录，否则录完就不可复现。
    """

    frame_count: int
    working_distance_mm: float | None = None
    registered: bool = False
    created: str | None = None
    extras: Mapping[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "frame_count": self.frame_count,
            "working_distance_mm": self.working_distance_mm,
            "registered": self.registered,
            "created": self.created,
        }
        payload.update(self.extras)
        return payload

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> SequenceMetadata:
        known = {"frame_count", "working_distance_mm", "registered", "created"}
        return cls(
            frame_count=int(data.get("frame_count", 0)),
            working_distance_mm=(
                None
                if data.get("working_distance_mm") is None
                else float(data["working_distance_mm"])
            ),
            registered=bool(data.get("registered", False)),
            created=data.get("created"),
            extras={k: v for k, v in data.items() if k not in known},
        )


@runtime_checkable
class FrameSource(Protocol):
    """帧来源。``analyze`` 走的就是这个接口，不关心帧从哪来。"""

    def frames(self) -> Iterator[DepthFrame]: ...

    def metadata(self) -> SequenceMetadata: ...


class PgmSequence:
    """从磁盘上的 PGM 序列读帧，可选带 ``metadata.json``。

    文件名按字典序排序后读入，因此 C++ 侧必须写零填充的 ``frame_%04d.pgm``，
    否则 ``frame_10`` 会排到 ``frame_2`` 前面。
    """

    def __init__(self, directory: str | Path, *, pattern: str = FRAME_GLOB) -> None:
        self.directory = Path(directory)
        if not self.directory.is_dir():
            raise FileNotFoundError(f"序列目录不存在: {self.directory}")
        self._paths = sorted(self.directory.glob(pattern))
        if not self._paths:
            raise FileNotFoundError(f"{self.directory} 下没有匹配 {pattern!r} 的帧")

    @property
    def paths(self) -> list[Path]:
        return list(self._paths)

    def __len__(self) -> int:
        return len(self._paths)

    def frames(self) -> Iterator[DepthFrame]:
        for index, path in enumerate(self._paths):
            yield DepthFrame(depth_mm=read_pgm(path), index=index)

    def metadata(self) -> SequenceMetadata:
        sidecar = self.directory / METADATA_FILENAME
        if sidecar.is_file():
            return SequenceMetadata.from_dict(json.loads(sidecar.read_text("utf-8")))
        # 没有旁路文件也是合法输入（手工摆放的 PGM 集合），但信息缺失要如实反映。
        return SequenceMetadata(frame_count=len(self._paths))


def write_sequence(
    directory: str | Path,
    frames: Iterator[np.ndarray] | list[np.ndarray],
    *,
    metadata: SequenceMetadata | None = None,
) -> Path:
    """把一组帧写成 ``PgmSequence`` 的目录布局。

    存在的主要理由是**测试夹具与合成数据**：让 ``PgmSequence`` 有确定的输入，
    从而 Seam 2 不需要相机就能验证。文件名格式与 C++ 侧保持一致。
    """
    from .pgm import write_pgm

    target = Path(directory)
    target.mkdir(parents=True, exist_ok=True)
    count = 0
    for count, frame in enumerate(frames, start=1):
        write_pgm(target / f"frame_{count - 1:04d}.pgm", frame)
    if metadata is None:
        metadata = SequenceMetadata(frame_count=count)
    (target / METADATA_FILENAME).write_text(
        json.dumps(metadata.as_dict(), indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return target
