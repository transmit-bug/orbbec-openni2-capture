"""命令行入口。

子命令对应调研的不同动作：

- ``record``   —— 调 C++ 采集二进制录一段序列（唯一需要相机的一步）
- ``analyze``  —— 分析一段序列，出风团指标与报告
- ``sweep``    —— 扫帧数，出"噪声底 / 可分辨高度 vs N"曲线
- ``synthetic``—— 生成合成序列，让人在不碰硬件的情况下把全流程跑一遍
- ``scene``    —— 2.5D 几何重建（不拟合基准面，深度值本身就是模型）
- ``preview``  —— 把 PGM 转成能直接打开的 PNG
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

from .accumulate import AccumulateParams, accumulate
from .analyze import analyze
from .frames import DepthFrame, PgmSequence, Roi, SequenceMetadata, write_sequence
from .geometry import (
    DEFAULT_DISCONTINUITY_MM,
    build_mesh,
    depth_to_points,
    orient_toward_viewer,
    render_overview,
    vertex_normals,
    write_ply,
)
from .intrinsics import Intrinsics
from .metrics import AnalyzeParams
from .pgm import read_pgm, write_pgm
from .preview import depth_to_rgb, render_depth_preview, render_sequence_preview
from .report import ReportInputs, render_report, render_sweep_table
from .sweep import noise_floor_curve
from .synthetic import SyntheticSpec
from .synthetic import frames as synthetic_frames

#: ``python/wheal/cli.py`` → 仓库根。
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAPTURE_BINARY = REPO_ROOT / "build" / "astra_capture"


def _parse_roi(text: str) -> Roi:
    parts = [int(value) for value in text.replace(" ", "").split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("ROI 需要 x,y,w,h 四个整数")
    return Roi(*parts)


def _load(
    source_dir: Path, limit: int | None
) -> tuple[list[DepthFrame], SequenceMetadata, PgmSequence]:
    source = PgmSequence(source_dir)
    frames: list[DepthFrame] = []
    for frame in source.frames():
        frames.append(frame)
        if limit is not None and len(frames) >= limit:
            break
    return frames, source.metadata(), source


def _params(args: argparse.Namespace) -> AnalyzeParams:
    params = AnalyzeParams()
    if args.mode:
        params = replace(params, average_mode=args.mode)
    if args.lateral_scale is not None:
        params = replace(params, lateral_scale_mm_per_px=args.lateral_scale)
    if args.threshold is not None:
        params = replace(params, height_threshold_mm=args.threshold)
    if args.quantum is not None:
        params = replace(params, quantum_mm=args.quantum)
    return params


def _full_roi(frames: list[DepthFrame]) -> Roi:
    height, width = frames[0].shape
    return Roi(0, 0, width, height)


def cmd_record(args: argparse.Namespace) -> int:
    """调用 C++ 采集二进制录制序列。

    采集刻意留在 C++：OpenNI2 设备访问、定制驱动、旧固件的
    ``ASTRA_DEPTH_FORMAT=ps`` 变通都在那边，跨语言重写没有收益。
    """
    binary = Path(args.binary)
    if not binary.is_file():
        print(
            f"找不到采集程序: {binary}\n"
            "先构建 C++ 侧：\n"
            "  cmake -S .. -B ../build && cmake --build ../build -j$(nproc)",
            file=sys.stderr,
        )
        return 2

    env = dict(os.environ)
    env.setdefault("ASTRA_DEPTH_FORMAT", "ps")
    env.setdefault("MALLOC_CHECK_", "0")
    env["LD_LIBRARY_PATH"] = f"{binary.parent}:{env.get('LD_LIBRARY_PATH', '')}"
    env["OPENNI2_DRIVERS_PATH"] = str(binary.parent / "OpenNI2" / "Drivers")

    command = [str(binary), "--record", str(args.frames), "--out", str(args.out)]
    if args.working_distance_mm is not None:
        command += ["--working-distance-mm", str(args.working_distance_mm)]
    print("$ " + " ".join(command))
    return subprocess.call(command, env=env)


def cmd_analyze(args: argparse.Namespace) -> int:
    frames, metadata, _ = _load(Path(args.sequence), args.frames)
    roi = args.roi or _full_roi(frames)
    params = _params(args)

    result = analyze(frames, roi, params)
    curve = noise_floor_curve(frames, roi, params) if args.out else []

    if args.out:
        report = render_report(
            ReportInputs(
                label=str(args.sequence),
                roi=roi,
                params=params,
                result=result,
                metadata=metadata,
                curve=curve,
                target_mm=args.target_mm,
            ),
            Path(args.out),
        )
        print(f"报告: {report}")
        print(render_sweep_table(curve))
    else:
        print(json.dumps(result.metrics.as_dict(), indent=2, ensure_ascii=False))

    for note in result.warnings:
        print(f"[warn] {note}", file=sys.stderr)
    return 0 if result.ok else 1


def cmd_sweep(args: argparse.Namespace) -> int:
    frames, metadata, _ = _load(Path(args.sequence), args.frames)
    roi = args.roi or _full_roi(frames)
    params = _params(args)

    result = analyze(frames, roi, params)
    curve = noise_floor_curve(frames, roi, params)
    print(render_sweep_table(curve))

    if args.out:
        report = render_report(
            ReportInputs(
                label=str(args.sequence),
                roi=roi,
                params=params,
                result=result,
                metadata=metadata,
                curve=curve,
                target_mm=args.target_mm,
            ),
            Path(args.out),
        )
        print(f"报告: {report}")
    return 0


def cmd_synthetic(args: argparse.Namespace) -> int:
    """生成合成序列 —— 让整条链路在没有任何硬件的情况下可端到端验证。"""
    spec = SyntheticSpec(
        noise_sigma_mm=args.noise,
        bump_height_mm=args.bump,
        bump_sigma_px=args.bump_sigma,
        hole_fraction=args.hole_fraction,
        seed=args.seed,
    )
    target = write_sequence(
        Path(args.out),
        synthetic_frames(spec, args.frames),
        metadata=SequenceMetadata(
            frame_count=args.frames,
            working_distance_mm=args.working_distance_mm,
            extras={"synthetic": True, "seed": args.seed},
        ),
    )
    print(f"合成序列: {target} ({args.frames} 帧, 噪声 sigma={args.noise} mm, 隆起={args.bump} mm)")
    return 0


def cmd_preview(args: argparse.Namespace) -> int:
    """把录制内容（PGM / PLY）转成能直接打开的 PNG。

    存在的理由：PGM 里的深度值是毫米原值（460..630），任何看图程序都只能看到
    高字节，也就是一张几乎全黑的图。这不是文件坏了，是看图的人少了一步归一化；
    这一步由这条命令替掉。
    """
    source = Path(args.input)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    if source.is_dir():
        sequence = PgmSequence(source)
        frames: list[DepthFrame] = []
        for frame in sequence.frames():
            frames.append(frame)
            if args.frames is not None and len(frames) >= args.frames:
                break
        if not frames:
            print(f"序列里没有帧: {source}", file=sys.stderr)
            return 2

        depths = [frame.depth_mm for frame in frames]
        # 缩略图数量封顶：120 帧全铺出来会得到一张 1 像素高的缩略图，等于没看。
        stride = args.stride or max(1, len(depths) // args.max_tiles)
        picked = depths[::stride][: args.max_tiles]
        sheet = render_sequence_preview(
            picked,
            out_dir / "sequence.png",
            title=(f"{source} —— {len(depths)} 帧，每 {stride} 帧取 1 帧共 {len(picked)} 张"),
        )
        first = render_depth_preview(
            depths[0],
            out_dir / "frame_0000.png",
            title=f"{source} 第 0 帧（黑 = 无数据）",
        )
        print(f"序列缩略图: {sheet}")
        print(f"首帧大图:   {first}")

        if args.fuse:
            fused = accumulate(frames, params=AccumulateParams(average_mode=args.mode))
            # 无效像素必须显式写成 0 再转 uint16：累积结果的无效位置可能是 NaN，
            # 直接 astype 会得到未定义的整数（且只弹一个 RuntimeWarning 就过去了）。
            write_pgm(
                out_dir / "fused.pgm",
                np.where(fused.valid, np.nan_to_num(fused.depth_mm), 0.0).astype(np.uint16),
            )
            fused_png = render_depth_preview(
                fused.depth_mm,
                out_dir / "fused.png",
                valid=fused.valid,
                title=(
                    f"累积 {fused.frame_count} 帧（覆盖率 "
                    f"{fused.per_frame_coverage * 100:.0f}% → {fused.fused_coverage * 100:.0f}%）"
                ),
            )
            print(f"累积图:     {fused_png}（同时写出 fused.pgm）")
        return 0

    depth = read_pgm(source)
    target = render_depth_preview(
        depth,
        out_dir / f"{source.stem}.png",
        title=f"{source.name}（黑 = 无数据）",
    )
    print(f"深度图: {target}")
    return 0


def cmd_scene(args: argparse.Namespace) -> int:
    """2.5D 场景建模：累积（降噪 + 填洞）→ 反投影 → 三角化 → 导出。

    与 ``analyze`` 不同，这里**不做基准面拟合**：室内场景有真实几何，
    深度值本身就是要建的模型，没有可以拟合掉的东西。
    """
    frames, metadata, _ = _load(Path(args.sequence), args.frames)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    fused = accumulate(frames)
    height, width = fused.depth_mm.shape
    intrinsics = (
        Intrinsics.from_hfov(args.hfov, width, height)
        if args.hfov is not None
        else Intrinsics.astra_pro_default(width, height)
    )

    render_overview(fused, intrinsics, out_dir / "overview.png", point_stride=args.point_stride)

    # 顶点色与法向是可选导出，但强烈建议开：没有它们的 PLY 在带光照的渲染器里
    # 打开就是一团黑（默认材质黑色 + 背面剔除），而“黑”同时也是合法的空数据，
    # 两种情况下屏幕上长得一模一样。
    colors = None
    if args.color:
        colors = depth_to_rgb(fused.depth_mm, fused.valid)[fused.valid]

    points = depth_to_points(fused.depth_mm, fused.valid, intrinsics)
    flat = points[fused.valid]
    write_ply(out_dir / "points.ply", flat, colors=colors, fmt=args.ply_format)

    mesh_summary: dict[str, object] = {"written": False}
    if not args.no_mesh:
        mesh = build_mesh(
            fused.depth_mm, fused.valid, intrinsics, discontinuity_mm=args.discontinuity
        )
        # 法向翻到指向相机（原点）的一侧。单视角 2.5D 面的法向本来朝 +z，
        # 也就是背离相机——不翻的话从相机方向看过去只有背面。
        normals = orient_toward_viewer(mesh.vertices, vertex_normals(mesh.vertices, mesh.faces))
        write_ply(
            out_dir / "mesh.ply",
            mesh.vertices,
            faces=mesh.faces,
            colors=colors,
            normals=normals,
            fmt=args.ply_format,
        )
        mesh_summary = {
            "written": True,
            "vertices": mesh.vertex_count,
            "faces": mesh.face_count,
            "extent_m": [float(v) for v in mesh.extent_m()],
            "vertex_colors": colors is not None,
            "vertex_normals": True,
            "ply_format": args.ply_format,
        }

    summary = {
        "sequence": str(args.sequence),
        "out": str(out_dir),
        "metadata": metadata.as_dict(),
        "fused": fused.statistics(),
        "intrinsics": {
            "fx": intrinsics.fx,
            "fy": intrinsics.fy,
            "cx": intrinsics.cx,
            "cy": intrinsics.cy,
            "calibrated": intrinsics.calibrated,
            "hfov_deg": intrinsics.hfov_deg,
            "vfov_deg": intrinsics.vfov_deg,
        },
        "mesh": mesh_summary,
    }
    (out_dir / "scene.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    stats = fused.statistics()
    print(
        f"累积 {stats['frame_count']} 帧: 覆盖 "
        f"{stats['per_frame_coverage'] * 100:.1f}% → {stats['fused_coverage'] * 100:.1f}% "
        f"({stats['coverage_gain']:.2f}x 填洞)"
    )
    print(
        f"深度 {stats['depth_min_mm']:.0f} .. {stats['depth_max_mm']:.0f} mm "
        f"(中位 {stats['depth_median_mm']:.0f})"
    )
    print(f"内参: {intrinsics.description()}")
    print(f"点云: {flat.shape[0]} 点 → {out_dir / 'points.ply'}")
    if mesh_summary.get("written"):
        extent = mesh_summary["extent_m"]
        print(
            f"网格: {mesh_summary['vertices']} 顶点 / {mesh_summary['faces']} 面 "
            f"→ {out_dir / 'mesh.ply'}"
        )
        print(f"包围盒: {extent[0]:.3f} x {extent[1]:.3f} x {extent[2]:.3f} m")
    if args.ply_format == "binary":
        print(
            "提示: 这份 PLY 是 binary。Blender 5.2.2 的二进制 PLY 读取器有 bug，"
            "打开会得到乱码点（或一团黑）——要在 Blender 里看请去掉 --ply-format binary。"
        )
    print(f"总览图: {out_dir / 'overview.png'}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="wheal",
        description="过敏原风团 2.5D 高度图分析（可行性调研）",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    def add_common(target: argparse.ArgumentParser) -> None:
        target.add_argument("sequence", help="PGM 序列目录")
        target.add_argument("--roi", type=_parse_roi, help="ROI，格式 x,y,w,h（默认整帧）")
        target.add_argument("--frames", type=int, help="最多读取多少帧")
        target.add_argument(
            "--mode",
            choices=["trimmed_mean", "median", "mean"],
            help="时间累积方式（默认 trimmed_mean）",
        )
        target.add_argument(
            "--lateral-scale",
            type=float,
            dest="lateral_scale",
            help="横向尺度 mm/px，提供后才给出面积与体积",
        )
        target.add_argument("--threshold", type=float, help="面积/体积的高度阈值 mm")
        target.add_argument("--quantum", type=float, help="传感器量化台阶 mm（默认 1.0）")
        target.add_argument("--out", help="报告输出目录")
        target.add_argument(
            "--target-mm",
            type=float,
            default=0.5,
            dest="target_mm",
            help="临床临时目标（默认 0.5，待确认）",
        )

    record = sub.add_parser("record", help="调 C++ 二进制录制深度序列")
    record.add_argument("--frames", type=int, default=300)
    record.add_argument("--out", required=True)
    record.add_argument("--binary", default=str(DEFAULT_CAPTURE_BINARY))
    record.add_argument(
        "--working-distance-mm", type=float, default=None, dest="working_distance_mm"
    )
    record.set_defaults(func=cmd_record)

    analyze_cmd = sub.add_parser("analyze", help="分析序列并输出指标/报告")
    add_common(analyze_cmd)
    analyze_cmd.set_defaults(func=cmd_analyze)

    sweep_cmd = sub.add_parser("sweep", help="扫帧数，输出核心曲线")
    add_common(sweep_cmd)
    sweep_cmd.set_defaults(func=cmd_sweep)

    synthetic_cmd = sub.add_parser("synthetic", help="生成合成序列")
    synthetic_cmd.add_argument("--out", required=True)
    synthetic_cmd.add_argument("--frames", type=int, default=64)
    synthetic_cmd.add_argument("--bump", type=float, default=0.8, help="隆起高度 mm")
    synthetic_cmd.add_argument("--bump-sigma", type=float, default=8.0, dest="bump_sigma")
    synthetic_cmd.add_argument("--noise", type=float, default=0.5, help="噪声 sigma mm")
    synthetic_cmd.add_argument("--hole-fraction", type=float, default=0.0, dest="hole_fraction")
    synthetic_cmd.add_argument(
        "--working-distance-mm", type=float, default=600.0, dest="working_distance_mm"
    )
    synthetic_cmd.add_argument("--seed", type=int, default=0)
    synthetic_cmd.set_defaults(func=cmd_synthetic)

    scene_cmd = sub.add_parser("scene", help="2.5D 场景建模：累积 → 点云 → 网格")
    scene_cmd.add_argument("sequence", help="PGM 序列目录")
    scene_cmd.add_argument("--out", required=True, help="输出目录")
    scene_cmd.add_argument("--frames", type=int, help="最多读取多少帧")
    scene_cmd.add_argument(
        "--mode",
        choices=["trimmed_mean", "median", "mean"],
        default="trimmed_mean",
        help="时间累积方式（默认 trimmed_mean）",
    )
    scene_cmd.add_argument(
        "--discontinuity",
        type=float,
        default=DEFAULT_DISCONTINUITY_MM,
        help=f"网格断开门限 mm（默认 {DEFAULT_DISCONTINUITY_MM:g}）",
    )
    scene_cmd.add_argument(
        "--hfov",
        type=float,
        default=None,
        help="水平视场角（度）。不给则用未标定的出厂近似焦距 570.3",
    )
    scene_cmd.add_argument("--point-stride", type=int, default=4, dest="point_stride")
    scene_cmd.add_argument("--no-mesh", action="store_true", dest="no_mesh")
    scene_cmd.add_argument(
        "--color",
        action="store_true",
        help="给 PLY 写顶点色（深度色标）。不开的话在带光照的渲染器里是黑的。",
    )
    scene_cmd.add_argument(
        "--ply-format",
        choices=["binary", "ascii"],
        default="ascii",
        dest="ply_format",
        help=(
            "PLY 编码。默认 ascii：Blender 5.2.2 的**二进制** PLY 读取器是错的（顶点元素"
            "里除了 x y z 还有任何别的属性，读到的位置就是错的），而 ascii 到处都能读。"
            "binary 体积约为 ascii 的 1/3，适合当存档。"
        ),
    )
    scene_cmd.set_defaults(func=cmd_scene)

    preview_cmd = sub.add_parser(
        "preview",
        help="PGM 深度图（单帧或序列）→ 能直接打开的 PNG",
        description=(
            "PGM 里的深度值是毫米原值，通用看图程序只能显示高字节，结果几乎全黑。"
            "这条命令重新归一化到有效区间并附上色标。"
        ),
    )
    preview_cmd.add_argument("input", help="单个 .pgm 文件，或一个 PGM 序列目录")
    preview_cmd.add_argument("--out", required=True, help="PNG 输出目录")
    preview_cmd.add_argument("--frames", type=int, help="序列最多读多少帧")
    preview_cmd.add_argument("--stride", type=int, help="缩略图抽帧步长（默认自动）")
    preview_cmd.add_argument(
        "--max-tiles", type=int, default=24, dest="max_tiles", help="缩略图最多几张（默认 24）"
    )
    preview_cmd.add_argument("--fuse", action="store_true", help="额外输出累积后的深度图")
    preview_cmd.add_argument(
        "--mode",
        choices=["trimmed_mean", "median", "mean"],
        default="trimmed_mean",
        help="--fuse 时的时间累积方式",
    )
    preview_cmd.set_defaults(func=cmd_preview)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
