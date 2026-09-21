"""调研报告生成。

调研的产出不是程序，是**结论**。所以这一步不是可有可无的收尾：没有报告，
前面所有数字都只是终端里的滚动输出。

报告刻意把"实测值"和"解析估计"分开写——``k × 噪声底`` 是估计，
``resolve_limit`` 的恢复实验才是实测。两者混在一起会让读者高估可信度。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # 必须在 pyplot 之前；报告生成不应要求图形环境

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .fonts import t as _t  # noqa: E402
from .frames import Roi, SequenceMetadata  # noqa: E402
from .metrics import AnalysisResult, AnalyzeParams  # noqa: E402
from .sweep import CurvePoint  # noqa: E402

#: 临床侧的临时目标。**待确认**，不是本调研可以单方面设定的要求。
PROVISIONAL_TARGET_MM = 0.5


@dataclass(frozen=True)
class ReportInputs:
    label: str
    roi: Roi
    params: AnalyzeParams
    result: AnalysisResult
    metadata: SequenceMetadata
    curve: list[CurvePoint]
    target_mm: float = PROVISIONAL_TARGET_MM


def plot_height_map(inputs: ReportInputs, path: Path) -> None:
    """高度残差图与基准面。人要能一眼看出隆起是真实的几何还是噪声。"""
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    height = inputs.result.height_map_mm

    image = axes[0].imshow(height, cmap="RdBu_r", interpolation="nearest")
    axes[0].set_title(
        _t(
            "高度残差 height_map (mm)\n正值 = 朝相机隆起",
            "height residual (mm)\npositive = toward camera",
        )
    )
    figure.colorbar(image, ax=axes[0], fraction=0.046)

    image = axes[1].imshow(inputs.result.baseline_mm, cmap="viridis", interpolation="nearest")
    axes[1].set_title(_t("拟合基准面 baseline (mm)", "fitted baseline (mm)"))
    figure.colorbar(image, ax=axes[1], fraction=0.046)

    image = axes[2].imshow(
        inputs.result.measurable_mask.astype(float), cmap="gray", interpolation="nearest"
    )
    axes[2].set_title(_t("可测像素掩码\n(有效 ∧ ¬飞点)", "measurable mask\n(valid and not flying)"))
    figure.colorbar(image, ax=axes[2], fraction=0.046)

    figure.tight_layout()
    figure.savefig(path, dpi=110)
    plt.close(figure)


def plot_curves(inputs: ReportInputs, path: Path) -> None:
    """噪声底与可分辨高度随帧数的变化 —— 调研的核心曲线。"""
    if not inputs.curve:
        return
    counts = [point.frame_count for point in inputs.curve]
    noise = [point.noise_floor_mm for point in inputs.curve]
    resolvable = [point.min_resolvable_mm for point in inputs.curve]

    figure, axes = plt.subplots(1, 2, figsize=(11, 4.2))

    axes[0].plot(counts, noise, "o-", label=_t("实测噪声底 sigma", "measured noise floor sigma"))
    axes[0].set_xscale("log", base=2)
    axes[0].set_yscale("log")
    axes[0].set_xlabel(_t("累积帧数 N", "accumulated frames N"))
    axes[0].set_ylabel(_t("噪声底 (mm)", "noise floor (mm)"))
    axes[0].set_title(_t("噪声底 vs 帧数", "noise floor vs frame count"))
    axes[0].grid(True, which="both", alpha=0.3)
    axes[0].legend()

    axes[1].plot(
        counts,
        resolvable,
        "o-",
        color="C3",
        label=_t("3 × 噪声底 (估计)", "3 x noise floor (estimate)"),
    )
    axes[1].axhline(
        inputs.target_mm,
        color="k",
        linestyle="--",
        label=_t(
            f"临时目标 {inputs.target_mm:g} mm（待临床确认）",
            f"provisional target {inputs.target_mm:g} mm",
        ),
    )
    if np.isfinite(inputs.result.metrics.peak_height_mm):
        axes[1].axhline(
            inputs.result.metrics.peak_height_mm,
            color="C0",
            linestyle=":",
            label=_t(
                f"本次测得峰值 {inputs.result.metrics.peak_height_mm:.3f} mm",
                f"measured peak {inputs.result.metrics.peak_height_mm:.3f} mm",
            ),
        )
    axes[1].set_xscale("log", base=2)
    axes[1].set_xlabel(_t("累积帧数 N", "accumulated frames N"))
    axes[1].set_ylabel(_t("可分辨的最小隆起 (mm)", "min resolvable height (mm)"))
    axes[1].set_title(_t("可分辨高度 vs 帧数", "resolvable height vs frame count"))
    axes[1].grid(True, which="both", alpha=0.3)
    axes[1].legend(fontsize=8)

    figure.tight_layout()
    figure.savefig(path, dpi=110)
    plt.close(figure)


def _verdict(inputs: ReportInputs) -> str:
    if not inputs.result.ok:
        return "**无法判定** —— 本次分析没有取得可用的高度测量（退化输入，见上方告警）。"
    resolvable = inputs.result.metrics.threshold_mm
    if resolvable <= inputs.target_mm:
        return (
            f"**有条件可行** —— 本次 3σ 门限 {resolvable:.3f} mm "
            f"≤ 临时目标 {inputs.target_mm:g} mm。仍需物理校验确认不是自洽的假象。"
        )
    return (
        f"**本次未达标** —— 3σ 门限 {resolvable:.3f} mm > 临时目标 {inputs.target_mm:g} mm。"
        "可尝试增加累积帧数或缩短工作距离。"
    )


def render_report(inputs: ReportInputs, out_dir: Path) -> Path:
    """写出 ``report.md`` 与两张图，返回报告路径。"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_height_map(inputs, out_dir / "height_map.png")
    plot_curves(inputs, out_dir / "curves.png")

    metrics = inputs.result.metrics
    lines: list[str] = []
    add = lines.append

    add(f"# 风团高度测量调研报告 —— {inputs.label}")
    add("")
    add("> 由 `wheal` 生成。这是**可行性调研**，不是临床结论。")
    add("")

    add("## 1. 结论")
    add("")
    add(_verdict(inputs))
    add("")

    add("## 2. 本次配置")
    add("")
    add("| 项目 | 值 |")
    add("| --- | --- |")
    add(f"| 序列 | `{inputs.label}` |")
    add(f"| 帧数 | {inputs.result.frame_count} |")
    add(f"| ROI | x={inputs.roi.x}, y={inputs.roi.y}, w={inputs.roi.w}, h={inputs.roi.h} |")
    add(f"| 工作距离 | {inputs.metadata.working_distance_mm} mm |")
    add(f"| 时间累积 | `{inputs.params.average_mode}` |")
    add(f"| 基准面阶数 | {inputs.params.baseline_degree} |")
    add(f"| 量化台阶 | {inputs.params.quantum_mm} mm |")
    add(f"| 飞点门限 | {inputs.params.flying_pixel_jump_mm} mm |")
    add("")

    add("## 3. 测量指标")
    add("")
    add("| 指标 | 值 |")
    add("| --- | --- |")
    add(f"| 峰值隆起 | {metrics.peak_height_mm:.4f} mm |")
    add(f"| 噪声底（基准面残差 sigma） | {metrics.noise_floor_mm:.4f} mm |")
    add(f"| 3σ 检出阈值 | {metrics.threshold_mm:.4f} mm |")
    add(f"| 有效像素比例 | {metrics.valid_ratio:.3f} |")
    add(f"| 可测像素比例 | {metrics.measurable_ratio:.3f} |")
    add(f"| 超阈值像素数 | {metrics.area_above_px} px |")
    area = (
        "未提供横向尺度" if metrics.area_above_mm2 is None else f"{metrics.area_above_mm2:.2f} mm²"
    )
    volume = "未提供横向尺度" if metrics.volume_mm3 is None else f"{metrics.volume_mm3:.3f} mm³"
    add(f"| 超阈值面积 | {area} |")
    add(f"| 超阈值体积 | {volume} |")
    add("")
    if metrics.area_above_mm2 is None:
        add(
            "> 面积与体积需要横向尺度（mm/px），本次未提供。隆起高度是纯 Z 方向的相对量，"
            "不需要相机内参；但把像素数换算成毫米需要横向尺度，缺失时不编造数字。"
        )
        add("")

    add("## 4. 噪声底 vs 帧数")
    add("")
    if inputs.curve:
        add("| N | 噪声底 (mm) | 3σ 可分辨 (mm) | 测得峰值 (mm) | 有效像素 |")
        add("| ---: | ---: | ---: | ---: | ---: |")
        for point in inputs.curve:
            add(
                f"| {point.frame_count} | {point.noise_floor_mm:.4f} | "
                f"{point.min_resolvable_mm:.4f} | {point.peak_height_mm:.4f} | "
                f"{point.valid_ratio:.3f} |"
            )
        add("")
        add("![curves](curves.png)")
    else:
        add("（未生成曲线）")
    add("")

    add("## 5. 高度残差图")
    add("")
    add("![height map](height_map.png)")
    add("")

    if inputs.result.warnings:
        add("## 6. 告警")
        add("")
        for note in inputs.result.warnings:
            add(f"- {note}")
        add("")

    add("## 7. 尚未验证的部分")
    add("")
    add(
        "**本报告的所有数字都来自这一台相机的一次采集。** 合成数据上的测试只能证明"
        "算法没写错，证明不了测量是真的。在把任何结论外推之前，至少还需要："
    )
    add("")
    add(
        "- **物理校验**：用千分尺独立测量过的已知高度参照物（塞尺 / 垫片 / 打印仿体），"
        "对比相机读数与真值。这是唯一能证明结论有效的环节。"
    )
    add("- **工作距离扫描**：0.6 / 0.7 / 0.8 / 1.0 m 各测一次，确定最佳机位与退化趋势。")
    add("- **适用人群边界**：深色皮肤、毛发、汗液、反光条件下的失效情况。")
    add("- **静止要求**：被试微动对结果的影响量级，据此给出固定方案。")
    add("- **时序同步**：Astra Pro 的深度与彩色走两个独立 USB 接口，运动时 D2C 会错位。")
    add("")
    add(
        "另需注意：传感器输出整数毫米，1 mm 量化台阶与风团高度同量级。本方法依赖"
        "传感器噪声充当抖动、并通过多帧平均突破台阶。若某次录制噪声过小（例如"
        "把相机固定得极稳、目标极均匀），抖动不足会导致量化台阶重新锁死结果。"
    )
    add("")

    report_path = out_dir / "report.md"
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report_path


def render_sweep_table(points: list[CurvePoint]) -> str:
    """仅返回曲线表格，供终端输出。"""
    lines = [f"{'N':>6} {'noise':>10} {'3sigma':>10} {'peak':>10} {'valid':>8}"]
    for point in points:
        lines.append(
            f"{point.frame_count:>6} {point.noise_floor_mm:>10.4f} "
            f"{point.min_resolvable_mm:>10.4f} {point.peak_height_mm:>10.4f} "
            f"{point.valid_ratio:>8.3f}"
        )
    return "\n".join(lines)
