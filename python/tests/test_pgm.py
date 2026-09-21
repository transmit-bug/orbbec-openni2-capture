"""PGM 读写的往返与格式一致性。

这些测试守的是一个**不会报错的**失败模式：字节序或 header 边界处理错了，
深度值会静默变成完全错误的读数，程序照常运行。
"""

from __future__ import annotations

import numpy as np
import pytest

from wheal.pgm import read_pgm, write_pgm


def test_roundtrip_16bit_preserves_every_value(tmp_path):
    rng = np.random.default_rng(0)
    image = rng.integers(0, 65536, size=(7, 11), dtype=np.uint16)
    path = tmp_path / "frame.pgm"

    write_pgm(path, image)

    assert np.array_equal(read_pgm(path), image)


def test_roundtrip_preserves_uint16_dtype(tmp_path):
    path = tmp_path / "frame.pgm"
    write_pgm(path, np.full((3, 3), 1234, dtype=np.uint16))
    assert read_pgm(path).dtype == np.uint16


def test_header_matches_the_cpp_writer_byte_for_byte(tmp_path):
    """C++ 侧 ``savePGM16`` 写的是 ``P5\\n{w} {h}\\n65535\\n`` 加大端 raster。"""
    path = tmp_path / "frame.pgm"
    write_pgm(path, np.zeros((2, 3), dtype=np.uint16))

    raw = path.read_bytes()
    header = b"P5\n3 2\n65535\n"
    assert raw.startswith(header)
    assert len(raw) == len(header) + 3 * 2 * 2


def test_raster_is_big_endian(tmp_path):
    """高字节在前。搞反了不会报错，只会把深度值变成另一个数字。"""
    path = tmp_path / "frame.pgm"
    write_pgm(path, np.array([[0x0102]], dtype=np.uint16))

    raster = path.read_bytes().split(b"65535\n", 1)[1]

    assert raster == b"\x01\x02"
    assert read_pgm(path)[0, 0] == 0x0102


def test_first_raster_byte_that_looks_like_whitespace_is_not_eaten(tmp_path):
    """header 与 raster 之间**只能**跳过一个空白字符。

    2560 = 0x0A00，其首字节 0x0A 恰好是换行符。若 reader 偷懒跳过"所有空白"，
    就会吃掉这一字节，把 2560 读成别的值——现象是图像整体偏了一格，很难查。
    """
    path = tmp_path / "frame.pgm"
    write_pgm(path, np.array([[2560, 300, 400]], dtype=np.uint16))

    assert read_pgm(path)[0, 0] == 2560


def test_header_comments_are_tolerated(tmp_path):
    path = tmp_path / "frame.pgm"
    path.write_bytes(b"P5\n# a comment\n2 1\n# another\n65535\n" + b"\x00\x01\x00\x02")

    assert np.array_equal(read_pgm(path), np.array([[1, 2]], dtype=np.uint16))


def test_8bit_pgm_is_promoted_to_uint16(tmp_path):
    path = tmp_path / "frame.pgm"
    path.write_bytes(b"P5\n2 1\n255\n" + bytes([10, 200]))

    image = read_pgm(path)

    assert image.dtype == np.uint16
    assert np.array_equal(image, np.array([[10, 200]], dtype=np.uint16))


def test_rejects_non_pgm(tmp_path):
    path = tmp_path / "frame.pgm"
    path.write_bytes(b"\x89PNG\r\n\x1a\n")
    with pytest.raises(ValueError, match="P5"):
        read_pgm(path)


def test_rejects_truncated_raster(tmp_path):
    path = tmp_path / "frame.pgm"
    path.write_bytes(b"P5\n4 4\n65535\n" + b"\x00\x01\x00\x01")
    with pytest.raises(ValueError, match="不完整"):
        read_pgm(path)


def test_rejects_wrong_dimensionality_on_write(tmp_path):
    with pytest.raises(ValueError, match="二维"):
        write_pgm(tmp_path / "frame.pgm", np.zeros((2, 2, 3), dtype=np.uint16))
