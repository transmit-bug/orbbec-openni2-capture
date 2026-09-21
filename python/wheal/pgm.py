"""16-bit PGM 读写。

刻意手写而不引依赖（imageio / pillow）：C++ 侧 ``src/main.cpp`` 写出的是大端、
``maxval=65535`` 的 ``P5``，格式已经固定。自己解析能保证两端逐字节一致。

**字节序是这里最容易错、且错了不会报错的地方** —— 它只会让深度值凭空变成
几百毫米的错误读数。因此有一条专门的 round-trip 测试，并且注意 header 与
raster 之间只允许**一个**空白字符：多跳一个字节会吃掉 raster 的首字节，
而 16-bit 数据的首字节完全可能是 ``0x0A``（恰好是换行符）。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

_WHITESPACE = b" \t\r\n\v\f"

MAXVAL_16BIT = 65535


def read_pgm(path: str | Path) -> np.ndarray:
    """读二进制 PGM（``P5``），返回 ``uint16`` 数组。

    8-bit 文件会被提升到 ``uint16``（值域不变），因为调研全程以 16-bit 为准。
    """
    raw = Path(path).read_bytes()
    if not raw.startswith(b"P5"):
        raise ValueError(f"不是二进制 PGM（期望 magic 'P5'）: {path}")

    pos = 2
    fields: list[int] = []
    while len(fields) < 3:
        if pos >= len(raw):
            raise ValueError(f"PGM header 不完整: {path}")
        char = raw[pos : pos + 1]
        if char in _WHITESPACE:
            pos += 1
        elif char == b"#":
            newline = raw.find(b"\n", pos)
            pos = len(raw) if newline == -1 else newline + 1
        else:
            start = pos
            while pos < len(raw) and raw[pos : pos + 1] not in _WHITESPACE:
                pos += 1
            fields.append(int(raw[start:pos]))

    width, height, maxval = fields
    if width <= 0 or height <= 0:
        raise ValueError(f"PGM 尺寸非法: {width}x{height}")
    if maxval <= 0 or maxval > MAXVAL_16BIT:
        raise ValueError(f"PGM maxval 非法: {maxval}")

    # header 与 raster 之间恰好一个空白字符。这里不能跳过全部空白。
    assert raw[pos : pos + 1] in _WHITESPACE, f"PGM header 缺少终止空白: {path}"
    pos += 1

    count = width * height
    bytes_per_sample = 1 if maxval < 256 else 2
    expected = count * bytes_per_sample
    available = len(raw) - pos
    if available < expected:
        raise ValueError(
            f"PGM raster 不完整: 需要 {expected} 字节，从 offset {pos} 起只有 {available}"
        )

    if maxval < 256:
        raster = np.frombuffer(raw, dtype=np.uint8, count=count, offset=pos)
        image = raster.reshape(height, width).astype(np.uint16)
    else:
        raster = np.frombuffer(raw, dtype=">u2", count=count, offset=pos)
        image = raster.reshape(height, width).astype(np.uint16)
    return image


def write_pgm(path: str | Path, image: np.ndarray, *, maxval: int = MAXVAL_16BIT) -> None:
    """写二进制 PGM，格式与 C++ 侧 ``savePGM16`` 完全一致（``P5`` + 大端）。"""
    array = np.asarray(image)
    if array.ndim != 2:
        raise ValueError(f"期望二维数组，收到 {array.ndim} 维")
    height, width = array.shape
    raster = array.astype(np.uint8).tobytes() if maxval < 256 else array.astype(">u2").tobytes()
    header = f"P5\n{width} {height}\n{maxval}\n".encode("ascii")
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(header + raster)
