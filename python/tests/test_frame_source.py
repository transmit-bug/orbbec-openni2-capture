"""Seam 2 —— 帧来源。

这些测试的存在理由就是**没有相机也能验证**。任何一条需要插着 Astra Pro 才能跑的
测试在这里都算失败的设计。
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from wheal.frames import (
    DepthFrame,
    PgmSequence,
    Roi,
    SequenceMetadata,
    valid_mask,
    write_sequence,
)


def _frames(count: int, shape=(6, 8)) -> list[np.ndarray]:
    return [np.full(shape, 600 + index, dtype=np.uint16) for index in range(count)]


def test_reads_back_a_written_sequence(tmp_path):
    write_sequence(tmp_path / "seq", _frames(4))

    source = PgmSequence(tmp_path / "seq")

    assert len(source) == 4
    loaded = list(source.frames())
    assert [int(f.depth_mm[0, 0]) for f in loaded] == [600, 601, 602, 603]
    assert [f.index for f in loaded] == [0, 1, 2, 3]


def test_frame_files_sort_numerically_not_lexically(tmp_path):
    """``frame_10`` 必须排在 ``frame_2`` 之后——零填充是为了这个。"""
    write_sequence(tmp_path / "seq", _frames(12))

    source = PgmSequence(tmp_path / "seq")

    assert [int(f.depth_mm[0, 0]) for f in source.frames()] == [600 + index for index in range(12)]


def test_metadata_roundtrip(tmp_path):
    metadata = SequenceMetadata(
        frame_count=3,
        working_distance_mm=612.5,
        registered=True,
        created="2026-01-01T00:00:00Z",
        extras={"note": "前臂内侧"},
    )
    write_sequence(tmp_path / "seq", _frames(3), metadata=metadata)

    loaded = PgmSequence(tmp_path / "seq").metadata()

    assert loaded.working_distance_mm == 612.5
    assert loaded.registered is True
    assert loaded.created == "2026-01-01T00:00:00Z"
    assert loaded.extras["note"] == "前臂内侧"


def test_missing_metadata_still_yields_a_usable_source(tmp_path):
    """手工摆放的 PGM 集合是合法输入，只是信息缺失要如实反映为 None。"""
    target = tmp_path / "seq"
    write_sequence(target, _frames(2))
    (target / "metadata.json").unlink()

    metadata = PgmSequence(target).metadata()

    assert metadata.frame_count == 2
    assert metadata.working_distance_mm is None


def test_metadata_json_is_readable_text(tmp_path):
    """录制之后人还要能直接看，所以是真 JSON 而不是二进制旁路。"""
    write_sequence(tmp_path / "seq", _frames(1))
    payload = json.loads((tmp_path / "seq" / "metadata.json").read_text("utf-8"))
    assert payload["frame_count"] == 1


def test_missing_directory_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        PgmSequence(tmp_path / "nope")


def test_empty_directory_raises(tmp_path):
    (tmp_path / "empty").mkdir()
    with pytest.raises(FileNotFoundError, match="没有匹配"):
        PgmSequence(tmp_path / "empty")


def test_pgm_sequence_satisfies_the_frame_source_protocol(tmp_path):
    from wheal.frames import FrameSource

    write_sequence(tmp_path / "seq", _frames(1))

    assert isinstance(PgmSequence(tmp_path / "seq"), FrameSource)


class TestValidMask:
    def test_zero_and_saturation_are_invalid(self):
        depth = np.array([[0, 100, 65535, 200]], dtype=np.uint16)
        assert valid_mask(depth).tolist() == [[False, True, False, True]]

    def test_max_valid_clamps_distant_returns(self):
        depth = np.array([[100, 900, 2000]], dtype=np.uint16)
        assert valid_mask(depth, 1500.0).tolist() == [[True, True, False]]


class TestRoi:
    def test_crop_selects_the_expected_region(self):
        array = np.arange(12, dtype=np.uint16).reshape(3, 4)
        assert Roi(1, 1, 2, 2).crop(array).tolist() == [[5, 6], [9, 10]]

    def test_roi_outside_the_frame_is_rejected(self):
        with pytest.raises(ValueError, match="超出帧范围"):
            Roi(0, 0, 10, 10).check_within((4, 4))

    def test_roundtrip_through_dict(self):
        assert Roi.from_dict(Roi(1, 2, 3, 4).as_dict()) == Roi(1, 2, 3, 4)


class TestDepthFrame:
    def test_rejects_non_2d(self):
        with pytest.raises(ValueError, match="二维"):
            DepthFrame(np.zeros((2, 2, 2), dtype=np.uint16))

    def test_rejects_wrong_dtype(self):
        with pytest.raises(ValueError, match="uint16"):
            DepthFrame(np.zeros((2, 2), dtype=np.float32))
