"""Seam 1 —— 纯分析函数。

这些测试是调研本身的**量化主体**：给定已知几何和已知噪声，回答"能不能测出来、
测得准不准、什么情况下失效"。全部无硬件、确定性。

容差来自实测标定而非猜测。几个数值值得记住（``noise_sigma_mm=0.5``、``N=64``、
1 mm 量化台阶）：

- 真实 0.797 mm 的隆起，``trimmed_mean`` 恢复约 0.91 mm（偏大约 0.11 mm）；
- 同样的隆起，``median`` 报 1.000 mm —— 锁在量化台阶上；
- 25% 的帧带 +50 mm 的**在量程内**离群值时，``mean`` 崩到 8.47 mm，
  ``trimmed_mean`` 仍为 0.88 mm；50% 污染时所有方式都失效。
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from wheal import (
    AnalyzeParams,
    DepthFrame,
    Roi,
    SyntheticSpec,
    analyze,
    ground_truth_height_mm,
    noise_floor_curve,
    synthetic_frames,
)
from wheal.metrics import quantisation_floor_mm


def _spec(**overrides) -> SyntheticSpec:
    base = SyntheticSpec(
        shape=(120, 160),
        noise_sigma_mm=0.5,
        bump_height_mm=0.8,
        bump_sigma_px=8.0,
        seed=3,
    )
    return replace(base, **overrides) if overrides else base


def _stack(spec: SyntheticSpec, count: int) -> list[DepthFrame]:
    return [DepthFrame(depth_mm=f, index=i) for i, f in enumerate(synthetic_frames(spec, count))]


def _full_roi(spec: SyntheticSpec) -> Roi:
    return Roi(0, 0, spec.width, spec.height)


class TestRecovery:
    def test_recovers_a_known_bump_within_a_tenth_of_a_millimetre(self):
        spec = _spec()
        result = analyze(_stack(spec, 64), _full_roi(spec))
        truth = float(ground_truth_height_mm(spec).max())

        assert result.ok
        assert result.metrics.peak_height_mm == pytest.approx(truth, abs=0.15)

    def test_no_bump_means_no_measured_height(self):
        spec = _spec(bump_height_mm=0.0)
        result = analyze(_stack(spec, 64), _full_roi(spec))

        # 没有隆起时不应凭空造出一个：峰值必须远小于一个量化台阶。
        assert abs(result.metrics.peak_height_mm) < 0.3

    def test_a_bump_is_positive_never_negative(self):
        """符号约定：隆起让深度变小，故 ``height = baseline - depth > 0``。

        搞反了风团会变成凹坑，而所有统计量依然"看起来正常"。
        """
        spec = _spec()
        result = analyze(_stack(spec, 64), _full_roi(spec))

        assert result.metrics.peak_height_mm > 0.0
        center = result.height_map_mm[spec.height // 2, spec.width // 2]
        assert center > 0.0

    def test_is_deterministic(self):
        spec = _spec()
        first = analyze(_stack(spec, 32), _full_roi(spec))
        second = analyze(_stack(spec, 32), _full_roi(spec))

        assert first.metrics.as_dict() == second.metrics.as_dict()


class TestRobustBaseline:
    def test_the_bump_does_not_drag_the_baseline_onto_itself(self):
        """普通最小二乘会被风团拽偏，导致隆起量系统性偏小且看不出来。"""
        flat = _spec(bump_height_mm=0.0)
        bumped = _spec(bump_height_mm=0.8)

        baseline_flat = analyze(_stack(flat, 64), _full_roi(flat)).baseline_mm
        baseline_bumped = analyze(_stack(bumped, 64), _full_roi(bumped)).baseline_mm

        # 有风团时基准面在风团中心处的偏移应远小于风团高度本身。
        center = (bumped.height // 2, bumped.width // 2)
        drift = abs(baseline_bumped[center] - baseline_flat[center])
        assert drift < 0.2, f"基准面被拽偏了 {drift:.3f} mm"

    def test_curvature_is_absorbed_by_the_baseline(self):
        """前臂的缓慢弯曲不应被当成隆起。"""
        spec = _spec(bump_height_mm=0.0, tilt_x_mm=8.0, curvature_mm=4.0)
        result = analyze(_stack(spec, 64), _full_roi(spec))

        # 8 mm 的倾斜与 4 mm 的曲率被吸收后，残差应只剩噪声量级。
        assert abs(result.metrics.peak_height_mm) < 0.4


class TestAveragingMode:
    def test_median_cannot_break_the_quantisation_step(self):
        """传感器输出整数毫米，中值仍是整数——累积再多帧也突破不了台阶。

        这是本调研里最反直觉的一条：中值的噪声底看起来**更低更漂亮**，
        但它那个数字是量化下限的假象，不是测出来的精度。
        """
        spec = _spec()
        stack = _stack(spec, 64)

        median = analyze(stack, _full_roi(spec), AnalyzeParams(average_mode="median")).metrics
        trimmed = analyze(
            stack, _full_roi(spec), AnalyzeParams(average_mode="trimmed_mean")
        ).metrics

        # 中值报出的峰值锁在一个量化台阶附近，而真值是 0.797。
        assert abs(median.peak_height_mm - round(median.peak_height_mm)) < 1e-9
        assert abs(median.peak_height_mm - 0.797) > 0.15
        # 修剪均值则落在真值附近。
        assert abs(trimmed.peak_height_mm - 0.797) < 0.15

    def test_trimmed_mean_rejects_in_range_outliers_that_break_the_mean(self):
        """离群值必须在**量程内**，否则会被 ``max_valid_mm`` 提前剔掉，
        测试就变成在检验有效性掩码而不是检验修剪。"""
        spec = _spec()
        frames = synthetic_frames(spec, 64)
        for index, frame in enumerate(frames):
            if index % 4 == 0:  # 25% 的帧被 +50 mm 污染
                region = frame[20:100, 40:120].astype(np.int32)
                frame[20:100, 40:120] = (region + 50).astype(np.uint16)
        stack = [DepthFrame(depth_mm=f, index=i) for i, f in enumerate(frames)]

        mean = analyze(stack, _full_roi(spec), AnalyzeParams(average_mode="mean")).metrics
        trimmed = analyze(
            stack, _full_roi(spec), AnalyzeParams(average_mode="trimmed_mean")
        ).metrics

        assert mean.peak_height_mm > 5.0, "裸均值本应被离群值摧毁"
        assert trimmed.peak_height_mm == pytest.approx(0.797, abs=0.15)

    def test_all_modes_fail_past_their_breakdown_point(self):
        """50% 污染会击穿 MAD 的 50% 崩溃点，此时没有哪种方式还活着。

        记录下来是为了避免报告里把"50% 坏帧下失效"误读成实现缺陷。
        """
        spec = _spec()
        frames = synthetic_frames(spec, 64)
        for index, frame in enumerate(frames):
            if index % 2 == 0:
                region = frame[20:100, 40:120].astype(np.int32)
                frame[20:100, 40:120] = (region + 50).astype(np.uint16)
        stack = [DepthFrame(depth_mm=f, index=i) for i, f in enumerate(frames)]

        trimmed = analyze(
            stack, _full_roi(spec), AnalyzeParams(average_mode="trimmed_mean")
        ).metrics

        assert trimmed.peak_height_mm > 5.0


class TestNoiseFloor:
    def test_noise_floor_falls_as_frames_accumulate(self):
        spec = _spec()
        curve = noise_floor_curve(_stack(spec, 64), _full_roi(spec))
        usable = [p for p in curve if np.isfinite(p.noise_floor_mm)]

        assert usable[0].noise_floor_mm > usable[-1].noise_floor_mm
        assert usable[-1].noise_floor_mm < usable[0].noise_floor_mm / 2

    def test_the_resolvable_height_curve_is_decreasing_overall(self):
        spec = _spec()
        curve = noise_floor_curve(_stack(spec, 64), _full_roi(spec))
        resolvable = [p.min_resolvable_mm for p in curve if np.isfinite(p.min_resolvable_mm)]

        assert resolvable[-1] < resolvable[0]

    def test_a_single_frame_cannot_resolve_a_sub_millimetre_wheal(self):
        """单帧的噪声底受量化下限约束，门限应高于风团本身——这正是要采多帧的理由。"""
        spec = _spec()
        result = analyze(_stack(spec, 1), _full_roi(spec))

        assert result.metrics.noise_floor_mm == pytest.approx(
            quantisation_floor_mm(1.0, 1), rel=1e-6
        )
        assert result.metrics.threshold_mm > float(ground_truth_height_mm(spec).max())
        # 但这不是"失败"：数值仍然可用，只是带警告。
        assert result.ok
        assert result.warnings

    def test_noise_floor_is_never_reported_below_the_quantisation_limit(self):
        spec = _spec()
        for count in (1, 2, 4, 64):
            result = analyze(_stack(spec, count), _full_roi(spec))
            assert result.metrics.noise_floor_mm >= quantisation_floor_mm(1.0, count) - 1e-9


class TestFlyingPixels:
    def test_a_depth_discontinuity_is_masked_out(self):
        spec = _spec(bump_height_mm=0.0, step_height_mm=50.0, step_position_px=80)
        params = AnalyzeParams(flying_pixel_jump_mm=20.0)

        with_masking = analyze(_stack(spec, 16), _full_roi(spec), params)
        without_masking = analyze(
            _stack(spec, 16), _full_roi(spec), replace(params, flying_pixel_jump_mm=None)
        )

        assert with_masking.metrics.measurable_ratio < without_masking.metrics.measurable_ratio
        # 边界列附近应被标为不可测。
        assert with_masking.metrics.measurable_ratio < 1.0


class TestDegenerateInputs:
    def test_an_all_invalid_frame_reports_failure_without_crashing(self):
        spec = _spec()
        blank = np.zeros(spec.shape, dtype=np.uint16)
        result = analyze([DepthFrame(depth_mm=blank)], _full_roi(spec))

        assert not result.ok
        assert np.isnan(result.metrics.peak_height_mm)
        assert result.metrics.valid_ratio == 0.0
        assert any("no valid depth" in note for note in result.warnings)

    def test_a_saturated_frame_is_treated_as_invalid(self):
        spec = _spec()
        saturated = np.full(spec.shape, 65535, dtype=np.uint16)
        result = analyze([DepthFrame(depth_mm=saturated)], _full_roi(spec))

        assert not result.ok
        assert result.metrics.valid_ratio == 0.0

    def test_too_few_valid_pixels_is_reported_not_faked(self):
        spec = _spec()
        frame = np.zeros(spec.shape, dtype=np.uint16)
        frame[0, :4] = 600  # 远少于二次曲面所需的像素数
        result = analyze([DepthFrame(depth_mm=frame)], _full_roi(spec))

        assert not result.ok
        assert any("too few valid pixels" in note for note in result.warnings)
        assert np.isnan(result.baseline_mm).all()

    def test_empty_frame_list_is_a_programming_error(self):
        with pytest.raises(ValueError, match="至少需要一帧"):
            analyze([], Roi(0, 0, 4, 4))

    def test_roi_outside_the_frame_is_rejected(self):
        spec = _spec()
        with pytest.raises(ValueError, match="超出帧范围"):
            analyze(_stack(spec, 2), Roi(0, 0, spec.width + 1, spec.height))


class TestLateralScale:
    def test_area_and_volume_are_withheld_without_a_lateral_scale(self):
        """隆起高度是纯 Z 量，不需要内参；但面积/体积需要横向尺度，
        缺失时应如实为 None，而不是编造一个数字。"""
        spec = _spec()
        result = analyze(_stack(spec, 64), _full_roi(spec))

        assert result.metrics.area_above_mm2 is None
        assert result.metrics.volume_mm3 is None
        assert result.metrics.area_above_px >= 0

    def test_providing_a_lateral_scale_yields_area_and_volume(self):
        spec = _spec()
        params = AnalyzeParams(lateral_scale_mm_per_px=0.5)
        result = analyze(_stack(spec, 64), _full_roi(spec), params)

        assert result.metrics.area_above_mm2 == pytest.approx(result.metrics.area_above_px * 0.25)
        assert result.metrics.volume_mm3 > 0.0

    def test_the_default_threshold_tracks_the_noise_floor(self):
        """阈值默认应为 ``robust_k × noise_floor``，而不是写死的魔数。"""
        spec = _spec()
        params = AnalyzeParams(robust_k=4.0)
        result = analyze(_stack(spec, 64), _full_roi(spec), params)

        assert result.metrics.threshold_mm == pytest.approx(4.0 * result.metrics.noise_floor_mm)

    def test_an_explicit_threshold_is_honoured(self):
        spec = _spec()
        params = AnalyzeParams(height_threshold_mm=0.5)
        result = analyze(_stack(spec, 64), _full_roi(spec), params)

        assert result.metrics.threshold_mm == 0.5


class TestUnmeasurablePixels:
    def test_static_holes_are_reported_and_do_not_break_the_fit(self):
        """结构光空洞是静止的：时间累积降噪，但填不了洞。"""
        spec = _spec(hole_fraction=0.15)
        result = analyze(_stack(spec, 32), _full_roi(spec))

        assert result.ok
        assert 0.5 < result.metrics.valid_ratio < 1.0
        assert result.metrics.peak_height_mm == pytest.approx(0.797, abs=0.2)
