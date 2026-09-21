"""2.5D 场景建模：内参、反投影、网格化、PLY 导出。

用合成数据验证几何正确性。**不测真实相机数据**——真实数据的质量取决于现场，
那是调研报告的内容，不是单元测试的断言对象。
"""

from __future__ import annotations

import struct

import numpy as np
import pytest

from wheal import (
    Intrinsics,
    PgmSequence,
    accumulate,
    build_mesh,
    depth_to_points,
    write_ply,
)
from wheal.frames import DepthFrame, write_sequence
from wheal.geometry import orient_toward_viewer, shaded_relief, vertex_normals
from wheal.intrinsics import ASTRA_PRO_FX
from wheal.synthetic import SyntheticSpec
from wheal.synthetic import frames as synthetic_frames


class TestIntrinsics:
    def test_default_scaling_follows_resolution(self):
        full = Intrinsics.astra_pro_default(640, 480)
        half = Intrinsics.astra_pro_default(320, 240)
        assert full.fx == pytest.approx(ASTRA_PRO_FX)
        assert half.fx == pytest.approx(ASTRA_PRO_FX / 2)
        assert not full.calibrated, "出厂近似值必须自我标记为未标定"

    def test_default_hfov_is_close_to_the_published_spec(self):
        """Astra Pro 标称水平视场角约 58.4°，出厂焦距应能复现它。"""
        assert Intrinsics.astra_pro_default().hfov_deg == pytest.approx(58.4, abs=0.5)

    def test_from_hfov_roundtrips(self):
        made = Intrinsics.from_hfov(58.4, 640, 480)
        assert made.hfov_deg == pytest.approx(58.4)

    def test_from_hfov_rejects_impossible_angles(self):
        for bad in (0.0, 180.0, -10.0):
            with pytest.raises(ValueError):
                Intrinsics.from_hfov(bad, 640, 480)

    def test_principal_point_unprojects_to_the_optical_axis(self):
        """主点上的像素应落在光轴上，即 x = y = 0。"""
        intr = Intrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
        depth = np.full((480, 640), 1000.0)
        points = intr.unproject(depth)
        assert points[240, 320, 0] == pytest.approx(0.0)
        assert points[240, 320, 1] == pytest.approx(0.0)
        assert points[240, 320, 2] == pytest.approx(1.0), "单位应为米"

    def test_unproject_applies_the_pinhole_relation(self):
        intr = Intrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0, width=640, height=480)
        points = intr.unproject(np.full((480, 640), 2000.0))
        # u=370, v=240, z=2 m => x = (370-320)*2/500 = 0.2
        assert points[240, 370, 0] == pytest.approx(0.2)
        assert points[240, 370, 2] == pytest.approx(2.0)

    def test_lateral_scale_shrinks_with_distance(self):
        intr = Intrinsics.astra_pro_default()
        assert intr.lateral_mm_per_px(1000.0) == pytest.approx(1000.0 / ASTRA_PRO_FX)
        assert intr.lateral_mm_per_px(2000.0) > intr.lateral_mm_per_px(1000.0)

    def test_unproject_rejects_a_mismatched_shape(self):
        with pytest.raises(ValueError, match="尺寸"):
            Intrinsics.astra_pro_default().unproject(np.zeros((10, 10)))

    def test_description_flags_uncalibrated(self):
        assert "未标定" in Intrinsics.astra_pro_default().description()
        calibrated = Intrinsics(1.0, 1.0, 1.0, 1.0, 2, 2, calibrated=True)
        assert "已标定" in calibrated.description()


class TestAccumulate:
    def test_accumulation_fills_dynamic_holes(self):
        """结构光匹配失败的洞是逐帧重掷的，累积应大幅提升覆盖率。

        用 ``hole_mode="dynamic"``：真实 Astra Pro 就是这种（实测相邻帧空洞掩码
        IoU 仅 0.497），120 帧把覆盖率从 43% 提到 94%。
        """
        spec = SyntheticSpec(
            shape=(60, 80),
            noise_sigma_mm=0.5,
            bump_height_mm=0.0,
            hole_fraction=0.4,
            hole_mode="dynamic",
            seed=4,
        )
        frames = [DepthFrame(f, i) for i, f in enumerate(synthetic_frames(spec, 64))]

        result = accumulate(frames)

        assert result.per_frame_coverage == pytest.approx(0.6, abs=0.05)
        assert result.fused_coverage > 0.95
        assert result.hole_fill_gain > 1.5

    def test_static_occlusion_holes_cannot_be_filled(self):
        """遮挡造成的洞在时间上是静止的，累积**填不了**。

        刻意记录这一条：填洞能力只对逐帧重掷的洞成立。若现场主要是遮挡，
        提高帧数没有帮助，只能改变机位。
        """
        spec = SyntheticSpec(shape=(60, 80), hole_fraction=0.25, hole_mode="static", seed=4)
        frames = [DepthFrame(f, i) for i, f in enumerate(synthetic_frames(spec, 64))]

        result = accumulate(frames)

        assert result.fused_coverage == pytest.approx(result.per_frame_coverage)
        assert result.hole_fill_gain == pytest.approx(1.0)

    def test_accumulate_rejects_empty_input(self):
        with pytest.raises(ValueError, match="frames 为空"):
            accumulate([])


class TestBuildMesh:
    def _plane(self, shape=(20, 30), depth=1000.0, step=None):
        depth_map = np.full(shape, depth, dtype=np.float64)
        if step is not None:
            depth_map[:, shape[1] // 2 :] = depth + step
        valid = np.ones(shape, dtype=bool)
        return depth_map, valid

    def test_a_plane_yields_two_triangles_per_quad(self):
        depth, valid = self._plane((20, 30))
        mesh = build_mesh(depth, valid, Intrinsics.astra_pro_default(30, 20))

        assert mesh.vertex_count == 20 * 30
        assert mesh.face_count == 19 * 29 * 2

    def test_faces_are_never_built_across_a_depth_step(self):
        """轮廓处前景与背景相差数十毫米；连面会拉出一层虚假的"裙边"。"""
        depth, valid = self._plane((20, 30), step=80.0)
        mesh = build_mesh(depth, valid, Intrinsics.astra_pro_default(30, 20), discontinuity_mm=25.0)

        # 19 行 x 29 列 = 551 个四边形；接缝所在的那一列（19 个四边形）全部被切断，
        # 剩 532 个四边形 -> 1064 张三角形。
        assert mesh.face_count == (19 * 29 - 19) * 2

    def test_raising_the_threshold_lets_faces_bridge_the_step(self):
        depth, valid = self._plane((20, 30), step=80.0)
        intr = Intrinsics.astra_pro_default(30, 20)
        tight = build_mesh(depth, valid, intr, discontinuity_mm=25.0)
        loose = build_mesh(depth, valid, intr, discontinuity_mm=200.0)

        assert loose.face_count > tight.face_count
        assert loose.face_count == 19 * 29 * 2

    def test_invalid_pixels_produce_no_vertices(self):
        depth, valid = self._plane((20, 30))
        valid[0, :] = False
        mesh = build_mesh(depth, valid, Intrinsics.astra_pro_default(30, 20))

        assert mesh.vertex_count == 19 * 30
        assert (mesh.vertex_index[0] == -1).all()

    def test_an_entirely_invalid_map_yields_an_empty_mesh(self):
        depth, _ = self._plane((10, 10))
        mesh = build_mesh(
            depth, np.zeros((10, 10), dtype=bool), Intrinsics.astra_pro_default(10, 10)
        )

        assert mesh.vertex_count == 0
        assert mesh.face_count == 0

    def test_mesh_vertices_match_the_unprojected_points(self):
        depth, valid = self._plane((12, 16))
        intr = Intrinsics.astra_pro_default(16, 12)
        mesh = build_mesh(depth, valid, intr)
        points = depth_to_points(depth, valid, intr)

        assert np.allclose(mesh.vertices, points[valid])

    def test_vertex_units_are_metres(self):
        depth, valid = self._plane((12, 16), depth=1500.0)
        mesh = build_mesh(depth, valid, Intrinsics.astra_pro_default(16, 12))
        assert mesh.vertices[:, 2] == pytest.approx(np.full(mesh.vertex_count, 1.5))


class TestWritePly:
    def _read_header(self, path):
        raw = path.read_bytes()
        end = raw.index(b"end_header\n") + len(b"end_header\n")
        return raw[:end].decode("ascii"), end, raw

    def test_header_declares_units_and_counts(self, tmp_path):
        vertices = np.zeros((5, 3), dtype=np.float64)
        path = write_ply(tmp_path / "p.ply", vertices)

        header, _, _ = self._read_header(path)
        assert header.startswith("ply\nformat binary_little_endian 1.0")
        assert "element vertex 5" in header
        assert "element face 0" in header
        assert "units: meters" in header

    def test_vertices_roundtrip_as_float32_big_endian_free(self, tmp_path):
        vertices = np.array([[1.0, 2.0, 3.0], [-0.5, 0.25, 1.75]], dtype=np.float64)
        path = write_ply(tmp_path / "p.ply", vertices)

        _, offset, raw = self._read_header(path)
        decoded = np.frombuffer(raw, dtype="<f4", count=6, offset=offset).reshape(2, 3)
        assert np.allclose(decoded, vertices, atol=1e-6)

    def test_faces_are_written_with_a_leading_count(self, tmp_path):
        vertices = np.zeros((3, 3), dtype=np.float64)
        faces = np.array([[0, 1, 2]], dtype=np.int32)
        path = write_ply(tmp_path / "p.ply", vertices, faces=faces)

        _, offset, raw = self._read_header(path)
        vertex_bytes = 3 * 3 * 4
        count = raw[offset + vertex_bytes]
        indices = struct.unpack_from("<3i", raw, offset + vertex_bytes + 1)
        assert count == 3
        assert indices == (0, 1, 2)

    def test_vertex_colour_optional_property_block(self, tmp_path):
        vertices = np.zeros((2, 3), dtype=np.float64)
        colors = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)
        path = write_ply(tmp_path / "p.ply", vertices, colors=colors)

        header, _, _ = self._read_header(path)
        assert "property uchar red" in header

    def test_rejects_bad_vertex_shape(self, tmp_path):
        with pytest.raises(ValueError, match=r"\(M,3\)"):
            write_ply(tmp_path / "p.ply", np.zeros((4, 2)))

    def test_rejects_mismatched_colour_count(self, tmp_path):
        with pytest.raises(ValueError, match="颜色数量"):
            write_ply(
                tmp_path / "p.ply",
                np.zeros((3, 3)),
                colors=np.zeros((2, 3), dtype=np.uint8),
            )

    def test_rejects_a_bad_format_name(self, tmp_path):
        """拼错格式名必须硬报错，不能默默退到某个默认值。"""
        with pytest.raises(ValueError, match="binary.*ascii"):
            write_ply(tmp_path / "p.ply", np.zeros((3, 3)), fmt="binaryy")

    def test_rejects_mismatched_normal_shape(self, tmp_path):
        with pytest.raises(ValueError, match="法向"):
            write_ply(tmp_path / "p.ply", np.zeros((3, 3)), normals=np.zeros((4, 3)))


class TestAsciiPly:
    """ASCII 输出 —— 存在的唯一理由是 Blender 读不了我们的二进制。

    属性区的顺序和二进制完全一致，所以这两组断言要能互相印证：如果哪天有人
    只改了一边，这里会挂。
    """

    def _vertices(self):
        return np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.6], [0.0, 0.2, 0.7]], dtype=np.float64)

    def _body(self, path) -> list[str]:
        """header 之后的所有行。不写死行号：行号会随 header 增删而失效。"""
        lines = path.read_text("ascii").splitlines()
        return lines[lines.index("end_header") + 1 :]

    def test_header_declares_ascii_and_the_same_property_block(self, tmp_path):
        path = write_ply(
            tmp_path / "p.ply",
            self._vertices(),
            faces=np.array([[0, 1, 2]], dtype=np.int32),
            colors=np.array([[1, 2, 3]] * 3, dtype=np.uint8),
            normals=np.tile([0.0, 0.0, -1.0], (3, 1)),
            fmt="ascii",
        )
        lines = path.read_text("ascii").splitlines()
        header = lines[: lines.index("end_header") + 1]

        assert header[0] == "ply"
        assert header[1] == "format ascii 1.0"
        assert header[4:] == [
            "element vertex 3",
            "property float x",
            "property float y",
            "property float z",
            "property float nx",
            "property float ny",
            "property float nz",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "element face 1",
            "property list uchar int vertex_indices",
            "end_header",
        ]

    def test_vertex_lines_carry_position_then_normal_then_colour(self, tmp_path):
        path = write_ply(
            tmp_path / "p.ply",
            self._vertices(),
            colors=np.array([[10, 20, 30]] * 3, dtype=np.uint8),
            normals=np.tile([0.0, 0.0, -1.0], (3, 1)),
            fmt="ascii",
        )
        body = self._body(path)

        assert body == [
            "0.000000 0.000000 0.500000 0.000000 0.000000 -1.000000 10 20 30",
            "0.100000 0.000000 0.600000 0.000000 0.000000 -1.000000 10 20 30",
            "0.000000 0.200000 0.700000 0.000000 0.000000 -1.000000 10 20 30",
        ]

    def test_ascii_roundtrips_vertices_and_faces(self, tmp_path):
        vertices = self._vertices()
        faces = np.array([[0, 1, 2]], dtype=np.int32)
        path = write_ply(tmp_path / "p.ply", vertices, faces=faces, fmt="ascii")
        body = self._body(path)

        decoded = np.array([[float(v) for v in line.split()] for line in body[:3]])
        assert np.allclose(decoded, vertices, atol=1e-6), "6 位小数在米制下是 1 微米"
        assert body[3] == "3 0 1 2"

    def test_binary_and_ascii_agree_on_the_property_block(self, tmp_path):
        """两条路径的属性区必须逐行一致。"""
        vertices = self._vertices()
        shared = {
            "faces": np.array([[0, 1, 2]], dtype=np.int32),
            "colors": np.array([[1, 2, 3]] * 3, dtype=np.uint8),
            "normals": np.tile([0.0, 0.0, -1.0], (3, 1)),
        }
        binary = write_ply(tmp_path / "b.ply", vertices, fmt="binary", **shared)
        ascii_ = write_ply(tmp_path / "a.ply", vertices, fmt="ascii", **shared)

        def block(path):
            raw = path.read_bytes()
            head = raw[: raw.index(b"end_header")].decode("ascii")
            return [line for line in head.splitlines() if line.startswith(("property", "element"))]

        assert block(binary) == block(ascii_)

    def test_no_face_lines_are_emitted_for_an_empty_face_array(self, tmp_path):
        path = write_ply(tmp_path / "p.ply", self._vertices(), fmt="ascii")

        assert self._body(path) == [
            "0.000000 0.000000 0.500000",
            "0.100000 0.000000 0.600000",
            "0.000000 0.200000 0.700000",
        ]


class TestPlyReaderInterop:
    """**回归测试：Blencer 5.2.2 的二进制 PLY 读取器是错的。**

    这不是猜的，是逐个实验做出来的（Blender 5.2.2 LTS，``wm.ply_import``，每个
    形状都独立复现过）。实测：只要 vertex 元素里除了 ``x y z`` 还有任何别的属性，
    Blender 读到的位置就是错的：

    - ``x y z foo``      → 读到 ``y z foo``（整体错一个属性）
    - ``x y z nx ny nz`` → 读到法向（单位向量，于是包围盒变成 ±1）
    - ``x y z red..blue``→ 垃圾 float（量级 1e38，等于没初始化的内存）

    ASCII 路径完全正常。这些断言不能验证 Blender（不能把 Blender 拉进单元测试），
    只能钉住我们这边的责任：**当顶点元素多于 3 个 float 时，别默认写二进制**。
    一旦有人“顺手统一一下格式”把 ASCII 拿掉，这份记录就是唯一的线索。
    """

    def test_ascii_is_the_only_layout_with_a_working_blender_path(self, tmp_path):
        vertices = np.array([[0.0, 0.0, 0.5], [0.1, 0.0, 0.6], [0.0, 0.2, 0.7]])
        ascii_path = write_ply(
            tmp_path / "a.ply",
            vertices,
            colors=np.array([[1, 2, 3]] * 3, dtype=np.uint8),
            fmt="ascii",
        )

        assert ascii_path.read_bytes().startswith(b"ply\nformat ascii 1.0\n")


class TestVertexNormals:
    def test_a_plane_has_a_constant_normal(self):
        vertices = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0]])
        faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)

        normals = vertex_normals(vertices, faces)

        assert np.allclose(np.linalg.norm(normals, axis=1), 1.0)
        assert np.allclose(np.abs(normals[:, 2]), 1.0), "在 z=const 平面上，法向只能是 ±z"
        assert np.allclose(normals, normals[0]), "平面上的顶点法向必须一致"

    def test_vertex_normals_are_unit_length_even_for_isolated_vertices(self):
        """没有任何面引用的顶点必须拿到一个占位方向，而不是 NaN。

        NaN 法向会让整块网格在渲染器里直接消失，比“朝向不对”更难查。
        """
        vertices = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [5.0, 5.0, 5.0]])
        faces = np.array([[0, 1, 2]], dtype=np.int32)

        normals = vertex_normals(vertices, faces)

        assert np.isfinite(normals).all()
        assert np.allclose(np.linalg.norm(normals, axis=1), 1.0)

    def test_no_faces_still_yields_usable_normals(self):
        normals = vertex_normals(np.zeros((4, 3)), np.zeros((0, 3), dtype=np.int32))

        assert np.isfinite(normals).all()
        assert np.allclose(np.linalg.norm(normals, axis=1), 1.0)


class TestOrientTowardViewer:
    def test_normals_pointing_away_are_flipped(self):
        """相机在原点、表面在 +z：法向必须是 -z 才看得见。"""
        vertices = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])
        normals = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])

        fixed = orient_toward_viewer(vertices, normals)

        assert np.allclose(fixed[:, 2], -1.0)

    def test_normals_already_facing_the_viewer_are_untouched(self):
        vertices = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]])
        normals = np.array([[0.0, 0.0, -1.0], [1.0, 0.0, -1.0]])

        assert np.allclose(orient_toward_viewer(vertices, normals), normals)

    def test_the_result_always_faces_the_viewer(self):
        """无论输入朝向如何，输出都要满足 n·(viewer - p) >= 0。"""
        rng = np.random.default_rng(7)
        vertices = rng.uniform(-1.0, 1.0, size=(64, 3)) + np.array([0.0, 0.0, 2.0])
        normals = rng.normal(size=(64, 3))
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)

        fixed = orient_toward_viewer(vertices, normals)

        assert (np.einsum("ij,ij->i", fixed, -vertices) >= 0).all()


class TestShadedRelief:
    def test_output_is_bounded_and_masked(self):
        spec = SyntheticSpec(shape=(24, 32), noise_sigma_mm=0.3, bump_height_mm=1.0, seed=2)
        frames = [DepthFrame(f, i) for i, f in enumerate(synthetic_frames(spec, 8))]
        fused = accumulate(frames)
        intr = Intrinsics.astra_pro_default(32, 24)
        points = depth_to_points(fused.depth_mm, fused.valid, intr)

        relief = shaded_relief(points, fused.valid)

        assert relief.shape == fused.valid.shape
        finite = relief[np.isfinite(relief)]
        assert finite.size > 0
        assert finite.min() >= 0.0 and finite.max() <= 1.0
        assert np.isnan(relief[~fused.valid]).all()


class TestSceneEndToEnd:
    def test_full_path_from_a_synthetic_sequence_on_disk(self, tmp_path):
        """Seam 2 → 融合 → 点云 → 网格 → PLY，全程无硬件。"""
        spec = SyntheticSpec(
            shape=(40, 60),
            noise_sigma_mm=0.8,
            bump_height_mm=2.0,
            hole_fraction=0.25,
            hole_mode="dynamic",
            seed=11,
        )
        write_sequence(tmp_path / "seq", synthetic_frames(spec, 24))

        frames = list(PgmSequence(tmp_path / "seq").frames())
        fused = accumulate(frames)
        intr = Intrinsics.astra_pro_default(60, 40)
        mesh = build_mesh(fused.depth_mm, fused.valid, intr)

        assert fused.hole_fill_gain > 1.2
        assert mesh.vertex_count == int(fused.valid.sum())
        assert mesh.face_count > 0
        assert write_ply(tmp_path / "mesh.ply", mesh.vertices, faces=mesh.faces).is_file()
