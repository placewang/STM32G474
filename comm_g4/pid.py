"""PID controller and interactive process simulator based on pid.c."""

from __future__ import annotations

import argparse
import csv
import math
from collections import deque
from dataclasses import dataclass, replace
from enum import IntEnum
from numbers import Real
from pathlib import Path
from typing import Any


class PIDMode(IntEnum):
    POSITION = 0
    INCREMENTAL = 1


class PIDStatus(IntEnum):
    OK = 0
    INVALID_ARGUMENT = -1
    TRACKING_LIMITED = 1


class PIDHandle:
    """Configuration and runtime state corresponding to PID_Handle_t."""

    def __init__(self) -> None:
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.Ts = 0.001
        self.out_max = 100.0
        self.out_min = -100.0
        self.output_rate_max = 0.0
        self.integral_max = 100.0
        self.integral_min = -100.0
        self.deadband = 0.0
        self.integral_threshold = 1e12
        self.derivative_filter = 0.8
        self.target_val = 0.0
        self.actual_val = 0.0
        self.actual_last = 0.0
        self.actual_prev = 0.0
        self.err = 0.0
        self.err_last = 0.0
        self.err_prev = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.out = 0.0
        self.last_delta = 0.0
        self.initialized = False
        self.mode = PIDMode.POSITION
        self.first_run = True


class PIDController:
    EPSILON = 1e-12
    REL_TOL = 1e-6

    @staticmethod
    def _valid_handle(pid: Any) -> bool:
        return isinstance(pid, PIDHandle) and pid.initialized

    @staticmethod
    def _is_finite(value: Any) -> bool:
        return isinstance(value, Real) and math.isfinite(value)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _nearly_equal(a: float, b: float) -> bool:
        scale = max(1.0, abs(a), abs(b))
        return abs(a - b) <= PIDController.REL_TOL * scale

    @staticmethod
    def init(pid: PIDHandle | None, mode: PIDMode, Ts: float) -> None:
        if not isinstance(pid, PIDHandle):
            return
        # PID_Init uses memset: a repeated initialization must discard all state.
        pid.__init__()
        pid.mode = PIDMode.INCREMENTAL if mode == PIDMode.INCREMENTAL else PIDMode.POSITION
        pid.Ts = float(Ts) if PIDController._is_finite(Ts) and Ts > 0.0001 else 0.001
        pid.initialized = True

    @staticmethod
    def set_param(pid: PIDHandle | None, kp: float, ki: float, kd: float) -> None:
        if not PIDController._valid_handle(pid) or not all(
            PIDController._is_finite(x) for x in (kp, ki, kd)
        ):
            return
        pid.Kp, pid.Ki, pid.Kd = float(kp), float(ki), float(kd)
        if abs(pid.Ki) < PIDController.EPSILON:
            pid.integral = 0.0

    @staticmethod
    def set_target(pid: PIDHandle | None, target: float) -> None:
        if PIDController._valid_handle(pid) and PIDController._is_finite(target):
            pid.target_val = float(target)

    @staticmethod
    def set_output_limit(pid: PIDHandle | None, max_val: float, min_val: float) -> None:
        if not PIDController._valid_handle(pid) or not all(
            PIDController._is_finite(x) for x in (max_val, min_val)
        ):
            return
        high, low = max(float(max_val), float(min_val)), min(float(max_val), float(min_val))
        pid.out_max, pid.out_min = high, low
        pid.out = PIDController._clamp(pid.out, low, high)

    @staticmethod
    def set_output_rate_limit(pid: PIDHandle | None, rate_max: float) -> None:
        if PIDController._valid_handle(pid) and PIDController._is_finite(rate_max):
            pid.output_rate_max = float(rate_max) if rate_max > 0.0 else 0.0

    @staticmethod
    def set_integral_limit(pid: PIDHandle | None, max_val: float, min_val: float) -> None:
        if not PIDController._valid_handle(pid) or not all(
            PIDController._is_finite(x) for x in (max_val, min_val)
        ):
            return
        high, low = max(float(max_val), float(min_val)), min(float(max_val), float(min_val))
        pid.integral_max, pid.integral_min = high, low
        pid.integral = PIDController._clamp(pid.integral, low, high)

    @staticmethod
    def set_deadband(pid: PIDHandle | None, deadband: float) -> None:
        if PIDController._valid_handle(pid) and PIDController._is_finite(deadband):
            pid.deadband = float(deadband) if deadband > 0.0 else 0.0

    @staticmethod
    def set_integral_threshold(pid: PIDHandle | None, threshold: float) -> None:
        if PIDController._valid_handle(pid) and PIDController._is_finite(threshold):
            pid.integral_threshold = float(threshold) if threshold > 0.0 else 1e12

    @staticmethod
    def set_derivative_filter(pid: PIDHandle | None, filter_coef: float) -> None:
        if PIDController._valid_handle(pid) and PIDController._is_finite(filter_coef):
            pid.derivative_filter = PIDController._clamp(float(filter_coef), 0.0, 1.0)

    @staticmethod
    def _apply_deadband(pid: PIDHandle, error: float) -> float:
        if abs(error) <= pid.deadband:
            return 0.0
        return error - math.copysign(pid.deadband, error)

    @staticmethod
    def _filter_derivative(pid: PIDHandle, raw: float) -> float:
        alpha = pid.derivative_filter
        pid.derivative = alpha * pid.derivative + (1.0 - alpha) * raw
        return pid.derivative

    @staticmethod
    def _apply_output_rate_limit(pid: PIDHandle, requested: float, previous: float) -> float:
        if pid.output_rate_max <= 0.0:
            return requested
        max_step = pid.output_rate_max * pid.Ts
        if not PIDController._is_finite(max_step) or max_step <= 0.0:
            return requested
        return PIDController._clamp(requested, previous - max_step, previous + max_step)

    @staticmethod
    def _calculate_position(pid: PIDHandle, actual_val: float) -> float:
        if not PIDController._is_finite(actual_val):
            return pid.out
        actual_val = float(actual_val)
        integral_old = pid.integral
        pid.actual_val = actual_val
        pid.err = PIDController._apply_deadband(pid, pid.target_val - actual_val)

        if pid.first_run:
            filtered_derivative = 0.0
        else:
            raw = -(actual_val - pid.actual_last) / pid.Ts
            filtered_derivative = PIDController._filter_derivative(pid, raw)

        if abs(pid.Ki) >= PIDController.EPSILON and abs(pid.err) < pid.integral_threshold:
            pid.integral = PIDController._clamp(
                pid.integral + pid.err * pid.Ts, pid.integral_min, pid.integral_max
            )

        candidate = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * filtered_derivative
        integral_delta_output = pid.Ki * (pid.integral - integral_old)
        if ((candidate > pid.out_max and integral_delta_output > 0.0) or
                (candidate < pid.out_min and integral_delta_output < 0.0)):
            pid.integral = integral_old
            candidate -= integral_delta_output
            integral_delta_output = 0.0

        limited = PIDController._apply_output_rate_limit(pid, candidate, pid.out)
        if ((limited < candidate and integral_delta_output > 0.0) or
                (limited > candidate and integral_delta_output < 0.0)):
            pid.integral = integral_old
            candidate -= integral_delta_output
            limited = PIDController._apply_output_rate_limit(pid, candidate, pid.out)

        pid.out = PIDController._clamp(limited, pid.out_min, pid.out_max)
        PIDController._update_history(pid, actual_val)
        return pid.out

    @staticmethod
    def _calculate_incremental(pid: PIDHandle, actual_val: float) -> float:
        if not PIDController._is_finite(actual_val):
            return 0.0
        actual_val = float(actual_val)
        pid.actual_val = actual_val
        pid.err = PIDController._apply_deadband(pid, pid.target_val - actual_val)
        derivative_delta = 0.0
        if pid.first_run:
            pid.actual_last = actual_val
            pid.actual_prev = actual_val
        else:
            previous_derivative = pid.derivative
            raw = -(actual_val - pid.actual_last) / pid.Ts
            current_derivative = PIDController._filter_derivative(pid, raw)
            derivative_delta = pid.Kd * (current_derivative - previous_derivative)

        delta = (pid.Kp * (pid.err - pid.err_last)
                 + pid.Ki * pid.Ts * pid.err + derivative_delta)
        old_output = pid.out
        delta = PIDController._apply_output_rate_limit(pid, old_output + delta, old_output) - old_output
        pid.out = PIDController._clamp(old_output + delta, pid.out_min, pid.out_max)
        applied_delta = pid.out - old_output
        PIDController._update_history(pid, actual_val)
        pid.last_delta = applied_delta
        return applied_delta

    @staticmethod
    def _update_history(pid: PIDHandle, actual_val: float) -> None:
        pid.err_prev, pid.err_last = pid.err_last, pid.err
        pid.actual_prev, pid.actual_last = pid.actual_last, actual_val
        pid.first_run = False

    @staticmethod
    def calculate(pid: PIDHandle | None, actual_val: float) -> float:
        if not PIDController._valid_handle(pid):
            return 0.0
        if pid.mode == PIDMode.POSITION:
            return PIDController._calculate_position(pid, actual_val)
        if pid.mode == PIDMode.INCREMENTAL:
            return PIDController._calculate_incremental(pid, actual_val)
        return pid.out

    @staticmethod
    def reset(pid: PIDHandle | None) -> None:
        if not PIDController._valid_handle(pid):
            return
        for name in ("actual_val", "actual_last", "actual_prev", "err", "err_last",
                     "err_prev", "integral", "derivative", "out", "last_delta"):
            setattr(pid, name, 0.0)
        pid.first_run = True

    @staticmethod
    def reset_tracking(pid: PIDHandle | None, actual_val: float,
                       current_output: float) -> PIDStatus:
        if (not PIDController._valid_handle(pid)
                or not PIDController._is_finite(actual_val)
                or not PIDController._is_finite(current_output)):
            return PIDStatus.INVALID_ARGUMENT
        actual_val, current_output = float(actual_val), float(current_output)
        tracked = PIDController._clamp(current_output, pid.out_min, pid.out_max)
        status = (PIDStatus.OK if PIDController._nearly_equal(tracked, current_output)
                  else PIDStatus.TRACKING_LIMITED)
        error = PIDController._apply_deadband(pid, pid.target_val - actual_val)
        pid.actual_val = pid.actual_last = pid.actual_prev = actual_val
        pid.err = pid.err_last = pid.err_prev = error
        pid.derivative, pid.out, pid.last_delta = 0.0, tracked, 0.0
        if pid.mode == PIDMode.POSITION and abs(pid.Ki) >= PIDController.EPSILON:
            requested = (tracked - pid.Kp * error) / pid.Ki
            pid.integral = PIDController._clamp(requested, pid.integral_min, pid.integral_max)
            if not PIDController._nearly_equal(pid.integral, requested):
                status = PIDStatus.TRACKING_LIMITED
        else:
            pid.integral = 0.0
        if pid.mode == PIDMode.POSITION:
            achievable = pid.Kp * error + pid.Ki * pid.integral
            if abs(achievable - tracked) > 1e-5:
                status = PIDStatus.TRACKING_LIMITED
        pid.first_run = False
        return status


@dataclass
class SimulationConfig:
    kp: float = 1.5
    ki: float = 0.5
    kd: float = 0.5
    sample_time: float = 0.05
    target: float = 5.0
    output_limit: float = 6.0
    rate_limit: float = 8.0
    integral_limit: float = 10.0
    deadband: float = 0.02
    integral_threshold: float = 6.0
    derivative_filter: float = 0.85
    process_gain: float = 1.0
    process_tau: float = 0.8
    process_delay: float = 0.10
    disturbance: float = 0.0
    duration: float = 20.0
    mode: PIDMode = PIDMode.POSITION


def simulate(config: SimulationConfig) -> dict[str, list[float]]:
    """Simulate a first-order-plus-dead-time process using the C PID behavior."""
    pid = PIDHandle()
    ctl = PIDController
    ctl.init(pid, config.mode, config.sample_time)
    ctl.set_param(pid, config.kp, config.ki, config.kd)
    ctl.set_target(pid, config.target)
    ctl.set_output_limit(pid, config.output_limit, -config.output_limit)
    ctl.set_output_rate_limit(pid, config.rate_limit)
    ctl.set_integral_limit(pid, config.integral_limit, -config.integral_limit)
    ctl.set_deadband(pid, config.deadband)
    ctl.set_integral_threshold(pid, config.integral_threshold)
    ctl.set_derivative_filter(pid, config.derivative_filter)

    sample_time = pid.Ts
    steps = min(100_000, max(2, int(config.duration / sample_time) + 1))
    delay_steps = min(steps, max(0, round(config.process_delay / sample_time)))
    delayed_commands = deque([0.0] * (delay_steps + 1), maxlen=delay_steps + 1)
    actual = command = 0.0
    result = {key: [] for key in
              ("time", "target", "actual", "output", "error", "integral", "p", "i", "d")}
    for step in range(steps):
        time_value = step * sample_time
        returned = ctl.calculate(pid, actual)
        command = returned if config.mode == PIDMode.POSITION else command + returned
        delayed_commands.append(command)
        process_input = delayed_commands[0]
        actual += ((config.process_gain * process_input + config.disturbance - actual)
                   / max(config.process_tau, 1e-6) * sample_time)
        result["time"].append(time_value)
        result["target"].append(config.target)
        result["actual"].append(actual)
        result["output"].append(command)
        result["error"].append(pid.err)
        result["integral"].append(pid.integral)
        result["p"].append(pid.Kp * pid.err)
        result["i"].append(pid.Ki * pid.integral)
        result["d"].append(pid.Kd * pid.derivative)
    return result


def response_metrics(data: dict[str, list[float]], target: float) -> dict[str, float]:
    """Return common PID Tuner step-response metrics."""
    time_values, actual = data["time"], data["actual"]
    if not time_values or abs(target) < 1e-12:
        return {"rise_time": math.nan, "settling_time": math.nan, "overshoot": 0.0,
                "steady_error": math.nan, "iae": math.nan, "peak": math.nan}
    direction = 1.0 if target > 0.0 else -1.0
    normalized = [direction * value for value in actual]
    magnitude = abs(target)

    def first_crossing(level: float) -> float:
        return next((time_values[i] for i, value in enumerate(normalized) if value >= level), math.nan)

    t10, t90 = first_crossing(0.1 * magnitude), first_crossing(0.9 * magnitude)
    rise_time = t90 - t10 if math.isfinite(t10) and math.isfinite(t90) else math.nan
    band = max(0.02 * magnitude, 1e-9)
    # The first sample after the final excursion is the settling time (O(n)).
    last_outside = -1
    for index, value in enumerate(actual):
        if abs(value - target) > band:
            last_outside = index
    settling_index = last_outside + 1
    settling_time = (time_values[settling_index]
                     if settling_index < len(time_values) else math.nan)
    peak = max(normalized)
    overshoot = max(0.0, (peak - magnitude) / magnitude * 100.0)
    steady_error = target - actual[-1]
    iae = sum(abs(error) for error in data["error"]) * (
        time_values[1] - time_values[0] if len(time_values) > 1 else 0.0
    )
    return {"rise_time": rise_time, "settling_time": settling_time,
            "overshoot": overshoot, "steady_error": steady_error,
            "iae": iae, "peak": direction * peak}


def imc_tune(config: SimulationConfig) -> tuple[float, float, float]:
    """Conservative IMC-style PID tuning for the configured FOPDT process."""
    gain = max(abs(config.process_gain), 1e-6)
    tau = max(config.process_tau, config.sample_time)
    delay = max(config.process_delay, config.sample_time)
    closed_loop_time = max(0.8 * tau, 2.0 * delay)
    kp = tau / (gain * (closed_loop_time + delay))
    integral_time = tau + 0.5 * delay
    derivative_time = tau * delay / (2.0 * tau + delay)
    ki = kp / integral_time
    kd = kp * derivative_time
    return kp, ki, kd


def tuning_score(config: SimulationConfig, data: dict[str, list[float]]) -> float:
    """Score a closed-loop response; lower is better."""
    metrics = response_metrics(data, config.target)
    scale = max(abs(config.target), 1.0)
    duration = max(config.duration, config.sample_time)
    steady = abs(metrics["steady_error"]) / scale
    overshoot = metrics["overshoot"] / 100.0
    settling = (metrics["settling_time"] / duration
                if math.isfinite(metrics["settling_time"]) else 1.5)
    iae = metrics["iae"] / (scale * duration)
    output_use = max(abs(value) for value in data["output"]) / max(config.output_limit, 1e-9)
    saturation_penalty = max(0.0, output_use - 0.98) * 2.0
    return 8.0 * steady + 2.5 * overshoot + settling + iae + saturation_penalty


def auto_design(config: SimulationConfig) -> tuple[SimulationConfig, SimulationConfig,
                                                     dict[str, float]]:
    """Design constraints, then refine PID gains using simulated response feedback."""
    gain = max(abs(config.process_gain), 1e-6)
    tau = max(config.process_tau, 1e-3)
    delay = max(config.process_delay, 0.0)
    target_scale = max(abs(config.target), 1.0)
    required_output = abs((config.target - config.disturbance) / gain)
    sample_time = max(0.001, min(0.1, tau / 30.0,
                                 delay / 6.0 if delay > 0.0 else tau / 20.0))
    duration = max(10.0, 12.0 * tau + 5.0 * delay)
    output_limit = max(1.0, 1.35 * required_output, 0.2 * target_scale)
    rate_limit = max(output_limit / max(tau, sample_time) * 3.0, output_limit / sample_time * 0.05)
    initial = replace(
        config,
        sample_time=sample_time,
        output_limit=output_limit,
        rate_limit=rate_limit,
        deadband=min(config.deadband, 0.002 * target_scale),
        integral_threshold=max(1.1 * target_scale, abs(config.target) + abs(config.disturbance)),
        derivative_filter=0.8 if delay <= tau else 0.9,
        duration=duration,
        mode=PIDMode.POSITION,
    )
    kp, ki, kd = imc_tune(initial)
    integral_limit = max(1.0, 1.5 * output_limit / max(ki, 1e-9))
    initial = replace(initial, kp=kp, ki=ki, kd=kd, integral_limit=integral_limit)
    initial_data = simulate(initial)

    best = initial
    best_data = initial_data
    best_score = tuning_score(best, best_data)
    evaluations = 1
    # Coordinate search uses each simulated response as feedback for the next pass.
    for factors in ((0.55, 0.75, 1.0, 1.3, 1.7),
                    (0.75, 0.9, 1.0, 1.12, 1.3),
                    (0.88, 0.96, 1.0, 1.05, 1.14)):
        for field in ("kp", "ki", "kd"):
            center = getattr(best, field)
            # A zero derivative remains a valid candidate but also gets a small trial value.
            base = center if center > 1e-12 else max(best.kp * best.sample_time, 1e-4)
            candidates = [0.0] + [base * factor for factor in factors] if field == "kd" else [base * factor for factor in factors]
            for value in candidates:
                trial = replace(best, **{field: value})
                if field == "ki":
                    trial = replace(trial, integral_limit=max(
                        1.0, 1.5 * trial.output_limit / max(value, 1e-9)
                    ))
                trial_data = simulate(trial)
                score = tuning_score(trial, trial_data)
                evaluations += 1
                if score < best_score:
                    best, best_data, best_score = trial, trial_data, score

    initial_score = tuning_score(initial, initial_data)
    final_metrics = response_metrics(best_data, best.target)
    report = {
        "initial_score": initial_score,
        "final_score": best_score,
        "evaluations": float(evaluations),
        "steady_error": final_metrics["steady_error"],
        "overshoot": final_metrics["overshoot"],
        "settling_time": final_metrics["settling_time"],
    }
    return initial, best, report


def refine_design(config: SimulationConfig, generation: int = 1) -> tuple[
        SimulationConfig, dict[str, float]]:
    """Continue tuning from the previous result with a progressively finer search."""
    best = config
    best_data = simulate(best)
    initial_score = tuning_score(best, best_data)
    best_score = initial_score
    evaluations = 1
    step = max(0.015, 0.18 / max(1, generation))
    factors = (1.0 - step, 1.0 - step / 2.0, 1.0,
               1.0 + step / 2.0, 1.0 + step)
    for _pass in range(2):
        for field in ("kp", "ki", "kd"):
            center = getattr(best, field)
            base = center if center > 1e-12 else max(best.kp * best.sample_time, 1e-4)
            candidates = [base * factor for factor in factors]
            if field == "kd":
                candidates.insert(0, 0.0)
            for value in candidates:
                trial = replace(best, **{field: value})
                if field == "ki":
                    trial = replace(trial, integral_limit=max(
                        trial.integral_limit,
                        1.5 * trial.output_limit / max(value, 1e-9),
                    ))
                trial_data = simulate(trial)
                score = tuning_score(trial, trial_data)
                evaluations += 1
                if score + 1e-12 < best_score:
                    best, best_data, best_score = trial, trial_data, score
    metrics = response_metrics(best_data, best.target)
    return best, {
        "initial_score": initial_score,
        "final_score": best_score,
        "evaluations": float(evaluations),
        "steady_error": metrics["steady_error"],
        "overshoot": metrics["overshoot"],
        "settling_time": metrics["settling_time"],
        "generation": float(generation),
    }


class LegacyPIDSimulator:
    """PID Toolbox-inspired desktop UI for controller design and comparison."""

    DEFAULTS = SimulationConfig()

    def __init__(self) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, RadioButtons, Slider, TextBox

        plt.rcParams["font.sans-serif"] = [
            "Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "DejaVu Sans"
        ]
        plt.rcParams["axes.unicode_minus"] = False
        self.plt, self.Slider = plt, Slider
        self.fig = plt.figure(figsize=(16, 9), facecolor="#eef1f4")
        self.fig.canvas.manager.set_window_title("PID Toolbox - 控制器设计与过程仿真")
        grid = self.fig.add_gridspec(2, 2, left=0.305, right=0.82, top=0.86,
                                     bottom=0.09, hspace=0.34, wspace=0.24)
        self.axes = [self.fig.add_subplot(grid[i, j]) for i in range(2) for j in range(2)]
        for axis in self.axes:
            axis.set_facecolor("#ffffff")
        self.baseline: dict[str, list[float]] | None = None
        self.baseline_label = ""
        self._suspend_redraw = False
        self._syncing_input = False
        self._animation_timer = None
        self._animation_backgrounds = None
        self._panel_backgrounds = None
        self._panel_limits = None
        self._redraw_timer = None
        self.animation_seconds = 3.0
        self.sliders: dict[str, Slider] = {}
        self.inputs: dict[str, TextBox] = {}
        specs = [
            ("kp", "比例 Kp", 0.0, 8.0), ("ki", "积分 Ki", 0.0, 5.0),
            ("kd", "微分 Kd", 0.0, 5.0), ("sample_time", "采样 Ts (s)", 0.005, 0.2),
            ("target", "设定值 Setpoint", -10.0, 10.0),
            ("output_limit", "输出限幅 Limit", 0.1, 20.0),
            ("rate_limit", "输出限速 Rate", 0.0, 30.0),
            ("integral_limit", "积分限幅 I-Limit", 0.0, 20.0),
            ("deadband", "死区 Deadband", 0.0, 1.0),
            ("integral_threshold", "积分分离 I-Threshold", 0.0, 10.0),
            ("derivative_filter", "微分滤波 D-Filter", 0.0, 1.0),
            ("process_gain", "过程增益 Gain", 0.1, 3.0),
            ("process_tau", "过程时常 Tau (s)", 0.1, 5.0),
            ("process_delay", "纯滞后 Delay (s)", 0.0, 2.0),
            ("disturbance", "外部扰动 Disturbance", -5.0, 5.0),
            ("duration", "仿真时长 Duration (s)", 5.0, 60.0),
        ]
        self.fig.text(0.025, 0.945, "控制器参数 / Controller", fontsize=10,
                      fontweight="bold", color="#243447")
        self.fig.text(0.025, 0.685, "约束与保护 / Constraints", fontsize=10,
                      fontweight="bold", color="#243447")
        self.fig.text(0.025, 0.397, "被控对象 / Plant Model", fontsize=10,
                      fontweight="bold", color="#243447")
        height = 0.022
        row_positions = [
            0.900, 0.857, 0.814, 0.771, 0.728,
            0.650, 0.607, 0.564, 0.521, 0.478, 0.435,
            0.362, 0.319, 0.276, 0.233, 0.190,
        ]
        for index, (name, label, low, high) in enumerate(specs):
            y_position = row_positions[index]
            axis = self.fig.add_axes([0.145, y_position, 0.085, height], facecolor="#dfe5ea")
            slider = Slider(axis, label, low, high, valinit=getattr(self.DEFAULTS, name),
                            color="#0076a8")
            slider.drawon = False
            slider.label.set_fontsize(7.5)
            slider.valtext.set_visible(False)
            input_axis = self.fig.add_axes([0.238, y_position - 0.001, 0.052, height + 0.002])
            text_input = TextBox(input_axis, "", initial=f"{getattr(self.DEFAULTS, name):g}",
                                 color="#ffffff", hovercolor="#f6f8fa")
            self._disable_broken_textbox_resize(text_input)
            text_input.text_disp.set_fontsize(8)
            slider.on_changed(lambda value, key=name: self._slider_changed(key, value))
            text_input.on_submit(lambda text, key=name: self._input_changed(key, text))
            self.sliders[name] = slider
            self.inputs[name] = text_input

        mode_axis = self.fig.add_axes([0.025, 0.065, 0.115, 0.07], facecolor="#eef1f4")
        self.mode = RadioButtons(mode_axis, ("位置式 Position", "增量式 Incremental"), active=0)
        self.mode.on_clicked(self._redraw)
        for label in self.mode.labels:
            label.set_fontsize(8)
        tune_axis = self.fig.add_axes([0.305, 0.905, 0.085, 0.042])
        baseline_axis = self.fig.add_axes([0.398, 0.905, 0.09, 0.042])
        reset_axis = self.fig.add_axes([0.496, 0.905, 0.075, 0.042])
        export_axis = self.fig.add_axes([0.579, 0.905, 0.085, 0.042])
        animation_axis = self.fig.add_axes([0.747, 0.912, 0.058, 0.03])
        self.fig.text(0.675, 0.925, "绘制时间 / Draw (s)", fontsize=8,
                      color="#243447", ha="left", va="center")
        self.animation_input = TextBox(animation_axis, "", initial="3.0",
                                       color="#ffffff", hovercolor="#f6f8fa")
        self._disable_broken_textbox_resize(self.animation_input)
        self.animation_input.text_disp.set_fontsize(8)
        self.animation_input.on_submit(self._animation_time_changed)
        self.tune_button = Button(tune_axis, "自动整定\nAuto Tune", color="#0076a8", hovercolor="#005f87")
        self.baseline_button = Button(baseline_axis, "保存基准\nStore Baseline",
                                      color="#dfe5ea", hovercolor="#cbd5dc")
        self.reset_button = Button(reset_axis, "复位\nReset", color="#dfe5ea", hovercolor="#cbd5dc")
        self.export_button = Button(export_axis, "导出 CSV\nExport", color="#dfe5ea", hovercolor="#cbd5dc")
        self.tune_button.on_clicked(self._auto_tune)
        self.baseline_button.on_clicked(self._store_baseline)
        self.reset_button.on_clicked(self._reset)
        self.export_button.on_clicked(self._export)
        self.metric_axis = self.fig.add_axes([0.84, 0.09, 0.14, 0.77], facecolor="#ffffff")
        self.metric_axis.set_xticks([])
        self.metric_axis.set_yticks([])
        for spine in self.metric_axis.spines.values():
            spine.set_color("#cbd5dc")
        self.data: dict[str, list[float]] = {}
        self._resize_callback_id = self.fig.canvas.mpl_connect("resize_event", self._on_resize)
        self._initialize_plots()
        self._initialize_metrics()
        self._redraw(None)
        self._capture_panel_backgrounds()

    @staticmethod
    def _disable_broken_textbox_resize(text_box: Any) -> None:
        """Work around Matplotlib 3.11 TextBox passing ResizeEvent to a mouse wrapper."""
        if not getattr(text_box, "_cids", None):
            return
        # TextBox registers resize_event last in Matplotlib 3.11.
        resize_cid = text_box._cids.pop()
        text_box.canvas.mpl_disconnect(resize_cid)

    def _on_resize(self, _event: object) -> None:
        if self._animation_timer is not None:
            self._animation_timer.stop()
            self._animation_timer = None
        self._animation_backgrounds = None
        for text_box in (*self.inputs.values(), self.animation_input):
            if text_box.capturekeystrokes:
                text_box.stop_typing()
                break

    def _config(self) -> SimulationConfig:
        values = {name: slider.val for name, slider in self.sliders.items()}
        values["mode"] = (PIDMode.POSITION if self.mode.value_selected == "位置式 Position"
                          else PIDMode.INCREMENTAL)
        return SimulationConfig(**values)

    def _slider_changed(self, name: str, value: float) -> None:
        if name in self.inputs:
            formatted = f"{value:.6g}"
            if self.inputs[name].text != formatted:
                # Updating the artist avoids a second widget draw and submit callback.
                self.inputs[name].text_disp.set_text(formatted)
        if not self._suspend_redraw:
            self._schedule_redraw()

    def _schedule_redraw(self) -> None:
        if self._redraw_timer is not None:
            self._redraw_timer.stop()
        timer = self.fig.canvas.new_timer(interval=40)
        timer.single_shot = True
        timer.add_callback(self._redraw, None)
        self._redraw_timer = timer
        timer.start()

    def _input_changed(self, name: str, text: str) -> None:
        if self._syncing_input:
            return
        slider = self.sliders[name]
        try:
            value = float(text)
        except ValueError:
            self.inputs[name].set_val(f"{slider.val:.6g}")
            return
        if not math.isfinite(value):
            self.inputs[name].set_val(f"{slider.val:.6g}")
            return
        nonnegative = {"sample_time", "output_limit", "rate_limit", "integral_limit",
                       "deadband", "integral_threshold", "derivative_filter",
                       "process_gain", "process_tau", "process_delay", "duration"}
        strictly_positive = {"sample_time", "output_limit", "process_gain", "process_tau", "duration"}
        if name in nonnegative:
            value = max(0.0, value)
        if name in strictly_positive:
            value = max(1e-6, value)
        if name == "derivative_filter":
            value = min(1.0, value)
        self._expand_slider(slider, value)
        slider.set_val(value)

    @staticmethod
    def _expand_slider(slider: Any, value: float) -> None:
        low, high = slider.valmin, slider.valmax
        if low <= value <= high:
            return
        span = max(high - low, abs(value), 1.0)
        if value < low:
            low = value - 0.1 * span
        if value > high:
            high = value + 0.1 * span
        slider.valmin, slider.valmax = low, high
        slider.ax.set_xlim(low, high)

    def _animation_time_changed(self, text: str) -> None:
        if self._syncing_input:
            return
        try:
            value = float(text)
        except ValueError:
            value = self.animation_seconds
        if not math.isfinite(value) or value <= 0.0:
            value = self.animation_seconds
        self.animation_seconds = min(value, 120.0)
        normalized = f"{self.animation_seconds:.6g}"
        if self.animation_input.text != normalized:
            self._syncing_input = True
            self.animation_input.set_val(normalized)
            self._syncing_input = False

    def _redraw(self, _event: object) -> None:
        self._redraw_timer = None
        self.data = simulate(self._config())
        self._render_data(len(self.data["time"]))

    def _initialize_plots(self) -> None:
        axes = self.axes
        titles = [
            "闭环响应 / Closed-loop Response", "控制器输出 / Controller Output",
            "控制偏差 / Control Error", "PID 分量 / PID Contributions",
        ]
        for axis, title in zip(axes, titles):
            axis.grid(True, color="#d7dde2", linewidth=0.7, alpha=0.8)
            axis.tick_params(labelsize=8)
            axis.set_title(title)
            axis.set_xlabel("时间 Time (s)", fontsize=8)
            axis.margins(x=0.02, y=0.18)
        self.plot_lines = {
            "baseline_actual": axes[0].plot([], [], color="#8b95a1", linestyle="--",
                                             linewidth=1.4, label="基准 Baseline")[0],
            "target": axes[0].plot([], [], "--", color="#c2413b", label="设定值 Setpoint")[0],
            "actual": axes[0].plot([], [], color="#167d73", linewidth=2,
                                    label="过程值 Process value")[0],
            "baseline_output": axes[1].plot([], [], color="#8b95a1", linestyle="--",
                                             linewidth=1.2, label="基准 Baseline")[0],
            "output": axes[1].plot([], [], color="#0076a8", label="当前 Current")[0],
            "upper_limit": axes[1].plot([], [], color="#c2413b", linestyle=":")[0],
            "lower_limit": axes[1].plot([], [], color="#c2413b", linestyle=":")[0],
            "error": axes[2].plot([], [], color="#7c3aed")[0],
            "zero": axes[2].axhline(0, color="#374151", linewidth=0.8),
            "p": axes[3].plot([], [], label="比例 P", color="#0076a8")[0],
            "i": axes[3].plot([], [], label="积分 I", color="#d97706")[0],
            "d": axes[3].plot([], [], label="微分 D", color="#c2413b")[0],
        }
        axes[0].legend(fontsize=8)
        axes[1].legend(fontsize=8)
        axes[3].legend(fontsize=8, ncol=3)

    def _initialize_metrics(self) -> None:
        axis = self.metric_axis
        self.metric_title = axis.text(0.08, 0.95, "性能指标\nPerformance",
                                      transform=axis.transAxes, fontsize=12,
                                      fontweight="bold", color="#243447", va="top")
        labels = ["上升时间 / Rise Time", "调节时间 / Settling", "超调量 / Overshoot",
                  "稳态偏差 / SS Error", "误差积分 / IAE", "响应峰值 / Peak"]
        self.metric_values = []
        y = 0.82
        for label in labels:
            axis.text(0.08, y, label, transform=axis.transAxes, fontsize=8, color="#586575")
            value_text = axis.text(0.08, y - 0.045, "", transform=axis.transAxes,
                                   fontsize=11, fontweight="bold", color="#0076a8")
            self.metric_values.append(value_text)
            y -= 0.125
        self.gain_summary = axis.text(0.08, 0.025, "", transform=axis.transAxes,
                                      fontsize=8, color="#243447")

    def _render_data(self, sample_count: int) -> None:
        sample_count = max(1, min(sample_count, len(self.data["time"])))
        shown = {key: values[:sample_count] for key, values in self.data.items()}
        t, axes = shown["time"], self.axes
        if self.baseline is not None:
            baseline_time = self.baseline["time"]
            self.plot_lines["baseline_actual"].set_data(baseline_time, self.baseline["actual"])
            self.plot_lines["baseline_output"].set_data(baseline_time, self.baseline["output"])
        else:
            self.plot_lines["baseline_actual"].set_data([], [])
            self.plot_lines["baseline_output"].set_data([], [])
        self.plot_lines["target"].set_data(t, shown["target"])
        self.plot_lines["actual"].set_data(t, shown["actual"])
        self.plot_lines["output"].set_data(t, shown["output"])
        limit = self._config().output_limit
        self.plot_lines["upper_limit"].set_data(t, [limit] * len(t))
        self.plot_lines["lower_limit"].set_data(t, [-limit] * len(t))
        self.plot_lines["error"].set_data(t, shown["error"])
        self.plot_lines["p"].set_data(t, shown["p"])
        self.plot_lines["i"].set_data(t, shown["i"])
        self.plot_lines["d"].set_data(t, shown["d"])
        axis_series = [
            (shown["target"], shown["actual"]),
            (shown["output"], [limit, -limit]),
            (shown["error"], [0.0]),
            (shown["p"], shown["i"], shown["d"]),
        ]
        for axis, series in zip(axes, axis_series):
            if not self._data_fits_axis(axis, t, series):
                axis.relim()
                axis.autoscale_view()
        metrics = response_metrics(shown, self._config().target)
        self._draw_metrics(metrics)
        self.fig.suptitle("PID Toolbox   控制器设计与过程仿真 / Controller Design & Process Simulation",
                          x=0.56, fontsize=14, fontweight="bold", color="#243447")
        self._draw_result_panels()

    @staticmethod
    def _data_fits_axis(axis: Any, time_values: list[float],
                        series: tuple[list[float], ...]) -> bool:
        if not time_values:
            return True
        x_low, x_high = axis.get_xlim()
        y_low, y_high = axis.get_ylim()
        values = [value for values in series for value in values if math.isfinite(value)]
        if not values:
            return True
        return (x_low <= time_values[0] and time_values[-1] <= x_high
                and y_low <= min(values) and max(values) <= y_high)

    def _draw_result_panels(self) -> None:
        """Draw only result panels, leaving the 34 parameter widgets untouched."""
        canvas = self.fig.canvas
        if self._panel_backgrounds is None or self._limits_changed():
            renderer = canvas.get_renderer()
            for axis in (*self.axes, self.metric_axis):
                axis.draw(renderer)
                canvas.blit(axis.bbox)
            self._capture_panel_backgrounds(redraw=False)
            canvas.flush_events()
            return
        dynamic_lines = [
            (self.axes[0], ("baseline_actual", "target", "actual")),
            (self.axes[1], ("baseline_output", "output", "upper_limit", "lower_limit")),
            (self.axes[2], ("error",)),
            (self.axes[3], ("p", "i", "d")),
        ]
        for index, (axis, names) in enumerate(dynamic_lines):
            canvas.restore_region(self._panel_backgrounds[index])
            for name in names:
                axis.draw_artist(self.plot_lines[name])
            canvas.blit(axis.bbox)
        canvas.restore_region(self._panel_backgrounds[4])
        for artist in (*self.metric_values, self.gain_summary):
            self.metric_axis.draw_artist(artist)
        canvas.blit(self.metric_axis.bbox)
        canvas.flush_events()

    def _limits_changed(self) -> bool:
        current = [(axis.get_xlim(), axis.get_ylim()) for axis in self.axes]
        return self._panel_limits != current

    def _capture_panel_backgrounds(self, redraw: bool = True) -> None:
        canvas = self.fig.canvas
        dynamic = [self.plot_lines[name] for name in
                   ("baseline_actual", "target", "actual", "baseline_output", "output",
                    "upper_limit", "lower_limit", "error", "p", "i", "d")]
        metric_artists = [*self.metric_values, self.gain_summary]
        for artist in (*dynamic, *metric_artists):
            artist.set_visible(False)
        if redraw:
            canvas.draw()
        else:
            renderer = canvas.get_renderer()
            for axis in (*self.axes, self.metric_axis):
                axis.draw(renderer)
        self._panel_backgrounds = [
            canvas.copy_from_bbox(axis.bbox) for axis in (*self.axes, self.metric_axis)
        ]
        self._panel_limits = [(axis.get_xlim(), axis.get_ylim()) for axis in self.axes]
        for artist in (*dynamic, *metric_artists):
            artist.set_visible(True)

    @staticmethod
    def _metric_value(value: float, unit: str = "") -> str:
        return f"{value:.3f}{unit}" if math.isfinite(value) else "未达到 / N/A"

    def _draw_metrics(self, metrics: dict[str, float]) -> None:
        values = [self._metric_value(metrics["rise_time"], " s"),
                  self._metric_value(metrics["settling_time"], " s"),
                  self._metric_value(metrics["overshoot"], " %"),
                  self._metric_value(metrics["steady_error"]),
                  self._metric_value(metrics["iae"]),
                  self._metric_value(metrics["peak"])]
        for text_artist, value in zip(self.metric_values, values):
            text_artist.set_text(value)
        config = self._config()
        self.gain_summary.set_text(
            f"Kp {config.kp:.3f}   Ki {config.ki:.3f}   Kd {config.kd:.3f}"
        )

    def _auto_tune(self, _event: object) -> None:
        kp, ki, kd = imc_tune(self._config())
        self._suspend_redraw = True
        try:
            for name, value in (("kp", kp), ("ki", ki), ("kd", kd)):
                slider = self.sliders[name]
                self._expand_slider(slider, value)
                slider.drawon = False
                slider.set_val(value)
                slider.drawon = True
        finally:
            self._suspend_redraw = False
        self.data = simulate(self._config())
        self._start_animation()

    def _start_animation(self) -> None:
        if self._animation_timer is not None:
            self._animation_timer.stop()
        total = len(self.data["time"])
        frames = min(24, total)
        interval_ms = max(40, int(self.animation_seconds * 1000 / frames))
        frame = 0
        # Compute metrics and axis limits once. Animation frames only update line artists.
        self._render_data(total)
        animated_names = ("target", "actual", "output", "upper_limit", "lower_limit",
                          "error", "p", "i", "d")
        for name in animated_names:
            self.plot_lines[name].set_visible(False)
        self.fig.canvas.draw()
        self._animation_backgrounds = [
            self.fig.canvas.copy_from_bbox(axis.bbox) for axis in self.axes
        ]
        for name in animated_names:
            self.plot_lines[name].set_visible(True)
        self._render_animation_frame(1)
        timer = self.fig.canvas.new_timer(interval=interval_ms)

        def advance() -> bool:
            nonlocal frame
            frame += 1
            count = min(total, math.ceil(total * frame / frames))
            self._render_animation_frame(count)
            if frame >= frames:
                timer.stop()
                self._animation_timer = None
                return False
            return True

        timer.add_callback(advance)
        self._animation_timer = timer
        timer.start()

    def _render_animation_frame(self, sample_count: int) -> None:
        count = max(1, min(sample_count, len(self.data["time"])))
        time_values = self.data["time"][:count]
        limit = self._config().output_limit
        updates = {
            "target": self.data["target"][:count], "actual": self.data["actual"][:count],
            "output": self.data["output"][:count], "upper_limit": [limit] * count,
            "lower_limit": [-limit] * count, "error": self.data["error"][:count],
            "p": self.data["p"][:count], "i": self.data["i"][:count],
            "d": self.data["d"][:count],
        }
        for name, values in updates.items():
            self.plot_lines[name].set_data(time_values, values)
        if self._animation_backgrounds is None:
            self.fig.canvas.draw_idle()
            return
        axis_lines = [
            ("baseline_actual", "target", "actual"),
            ("baseline_output", "output", "upper_limit", "lower_limit"),
            ("error",),
            ("p", "i", "d"),
        ]
        for axis, background, names in zip(self.axes, self._animation_backgrounds, axis_lines):
            self.fig.canvas.restore_region(background)
            for name in names:
                axis.draw_artist(self.plot_lines[name])
            self.fig.canvas.blit(axis.bbox)
        self.fig.canvas.flush_events()

    def _store_baseline(self, _event: object) -> None:
        self.baseline = {key: values.copy() for key, values in self.data.items()}
        config = self._config()
        self.baseline_label = f"Kp={config.kp:.2f}, Ki={config.ki:.2f}, Kd={config.kd:.2f}"
        self._redraw(None)

    def _reset(self, _event: object) -> None:
        self.baseline = None
        self.baseline_label = ""
        self._suspend_redraw = True
        try:
            for name, slider in self.sliders.items():
                default = getattr(self.DEFAULTS, name)
                self._expand_slider(slider, default)
                slider.drawon = False
                slider.set_val(default)
                slider.drawon = True
        finally:
            self._suspend_redraw = False
        self.mode.set_active(0)
        self._redraw(None)

    def _export(self, _event: object) -> None:
        path = Path.cwd() / "pid_simulation_result.csv"
        keys = list(self.data)
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(keys)
            writer.writerows(zip(*(self.data[key] for key in keys)))
        self.fig.suptitle(f"数据已导出 / Exported: {path}", fontsize=12, color="#167d73")
        self.fig.canvas.draw_idle()

    def show(self) -> None:
        self.plt.show()


class PIDSimulator:
    """Responsive Tkinter UI. Matplotlib is used only as a plotting surface."""

    DEFAULTS = SimulationConfig()

    PARAMS = [
        ("kp", "比例 Kp", 0.0, 8.0, "controller"),
        ("ki", "积分 Ki", 0.0, 5.0, "controller"),
        ("kd", "微分 Kd", 0.0, 5.0, "controller"),
        ("sample_time", "采样 Ts (s)", 0.005, 0.2, "controller"),
        ("target", "设定值 Setpoint", -10.0, 10.0, "controller"),
        ("output_limit", "输出限幅 Limit", 0.1, 20.0, "constraints"),
        ("rate_limit", "输出限速 Rate", 0.0, 30.0, "constraints"),
        ("integral_limit", "积分限幅 I-Limit", 0.0, 20.0, "constraints"),
        ("deadband", "死区 Deadband", 0.0, 1.0, "constraints"),
        ("integral_threshold", "积分分离 I-Threshold", 0.0, 20.0, "constraints"),
        ("derivative_filter", "微分滤波 D-Filter", 0.0, 1.0, "constraints"),
        ("process_gain", "过程增益 Gain", 0.1, 5.0, "plant"),
        ("process_tau", "过程时常 Tau (s)", 0.1, 10.0, "plant"),
        ("process_delay", "纯滞后 Delay (s)", 0.0, 5.0, "plant"),
        ("disturbance", "外部扰动 Disturbance", -10.0, 10.0, "plant"),
        ("duration", "仿真时长 Duration (s)", 5.0, 120.0, "plant"),
    ]

    def __init__(self) -> None:
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
        import matplotlib as mpl

        self.tk, self.ttk = tk, ttk
        # TkAgg does not inherit the font setup from the legacy Matplotlib UI.
        # Pick installed CJK fonts explicitly so chart labels do not fall back to DejaVu Sans.
        mpl.rcParams["font.sans-serif"] = [
            "Microsoft YaHei", "SimHei", "Microsoft JhengHei", "Noto Sans CJK SC",
            "Arial", "DejaVu Sans"
        ]
        mpl.rcParams["axes.unicode_minus"] = False
        self.root = tk.Tk()
        self.root.title("PID Toolbox - 控制器设计与过程仿真")
        self.root.geometry("1500x880")
        self.root.minsize(1180, 720)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")

        self.vars: dict[str, tk.DoubleVar] = {}
        self.scales: dict[str, ttk.Scale] = {}
        self.entries: dict[str, ttk.Entry] = {}
        self._after_id = None
        self._animation_id = None
        self._updating = False
        self.baseline: dict[str, list[float]] | None = None
        self.data: dict[str, list[float]] = {}
        self._tuning_signature: tuple[float, ...] | None = None
        self._tuning_generation = 0

        toolbar = ttk.Frame(self.root, padding=(8, 6))
        toolbar.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(toolbar, text="PID Toolbox", font=("Microsoft YaHei", 15, "bold")).pack(side=tk.LEFT, padx=(4, 18))
        ttk.Button(toolbar, text="自动整定 Auto Tune", command=self._auto_tune).pack(side=tk.LEFT, padx=3)
        ttk.Button(toolbar, text="保存基准 Baseline", command=self._store_baseline).pack(side=tk.LEFT, padx=3)
        ttk.Button(toolbar, text="复位 Reset", command=self._reset).pack(side=tk.LEFT, padx=3)
        ttk.Button(toolbar, text="导出 CSV", command=self._export).pack(side=tk.LEFT, padx=3)
        ttk.Label(toolbar, text="绘制时间 Draw (s)").pack(side=tk.LEFT, padx=(18, 4))
        self.draw_seconds = tk.StringVar(value="1.5")
        self._valid_draw_seconds = 1.5
        draw_entry = ttk.Entry(toolbar, textvariable=self.draw_seconds, width=7)
        draw_entry.pack(side=tk.LEFT)
        draw_entry.bind("<Return>", self._validate_draw_seconds)
        draw_entry.bind("<FocusOut>", self._validate_draw_seconds)
        self.status_var = tk.StringVar(value="就绪 / Ready")
        ttk.Label(toolbar, textvariable=self.status_var).pack(side=tk.RIGHT, padx=8)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        controls = ttk.Frame(body, width=400)
        controls.pack_propagate(False)
        plots = ttk.Frame(body)
        body.add(controls, weight=0)
        body.add(plots, weight=1)

        canvas = tk.Canvas(controls, highlightthickness=0, width=400)
        scrollbar = ttk.Scrollbar(controls, orient=tk.VERTICAL, command=canvas.yview)
        self.control_inner = ttk.Frame(canvas, padding=(4, 2))
        self.control_inner.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        self._control_window = canvas.create_window(
            (0, 0), window=self.control_inner, anchor="nw", width=378
        )
        canvas.bind(
            "<Configure>",
            lambda event: canvas.itemconfigure(
                self._control_window, width=max(360, event.width - scrollbar.winfo_reqwidth())
            ),
        )
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        groups = {
            "controller": ttk.LabelFrame(self.control_inner, text="控制器 / Controller", padding=7),
            "constraints": ttk.LabelFrame(self.control_inner, text="约束保护 / Constraints", padding=7),
            "plant": ttk.LabelFrame(self.control_inner, text="被控对象 / Plant", padding=7),
        }
        for frame in groups.values():
            frame.pack(fill=tk.X, pady=4)
        for name, label, low, high, group in self.PARAMS:
            self._add_parameter(groups[group], name, label, low, high)

        mode_frame = ttk.LabelFrame(self.control_inner, text="控制模式 / Mode", padding=7)
        mode_frame.pack(fill=tk.X, pady=4)
        self.mode_var = tk.StringVar(value="position")
        ttk.Radiobutton(mode_frame, text="位置式 Position", value="position",
                        variable=self.mode_var, command=self._schedule_update).pack(side=tk.LEFT)
        ttk.Radiobutton(mode_frame, text="增量式 Incremental", value="incremental",
                        variable=self.mode_var, command=self._schedule_update).pack(side=tk.LEFT, padx=8)

        self.figure = Figure(figsize=(10, 7), dpi=100, facecolor="#eef1f4")
        self.axes = self.figure.subplots(2, 2)
        self.figure.subplots_adjust(left=0.07, right=0.98, top=0.91, bottom=0.08,
                                    hspace=0.34, wspace=0.24)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plots)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.metric_var = tk.StringVar()
        ttk.Label(plots, textvariable=self.metric_var, anchor="center",
                  font=("Microsoft YaHei", 9)).pack(fill=tk.X, pady=(2, 0))
        self._init_plot_lines()
        self._update_now()

    def _add_parameter(self, parent: Any, name: str, label: str,
                       low: float, high: float) -> None:
        ttk = self.ttk
        row = ttk.Frame(parent)
        row.pack(fill=self.tk.X, pady=2)
        row.columnconfigure(0, minsize=145)
        row.columnconfigure(1, weight=1, minsize=90)
        row.columnconfigure(2, minsize=105)
        ttk.Label(row, text=label, anchor="w").grid(row=0, column=0, sticky="w", padx=(0, 5))
        variable = self.tk.DoubleVar(value=getattr(self.DEFAULTS, name))
        scale = ttk.Scale(row, from_=low, to=high, variable=variable,
                          command=lambda value, key=name: self._scale_changed(key, value))
        scale.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        entry = ttk.Entry(row, width=12, justify="right")
        entry.insert(0, f"{getattr(self.DEFAULTS, name):g}")
        entry.grid(row=0, column=2, sticky="e")
        entry.bind("<KeyRelease>", lambda _event, key=name: self._entry_edited(key))
        entry.bind("<Return>", lambda _event, key=name: self._entry_changed(key))
        entry.bind("<FocusOut>", lambda _event, key=name: self._entry_changed(key))
        self.vars[name], self.scales[name], self.entries[name] = variable, scale, entry

    def _scale_changed(self, name: str, value: str) -> None:
        if self._updating:
            return
        entry = self.entries[name]
        entry.delete(0, self.tk.END)
        entry.insert(0, f"{float(value):.6g}")
        self._schedule_update()

    def _entry_changed(self, name: str) -> None:
        entry, scale = self.entries[name], self.scales[name]
        try:
            value = float(entry.get())
        except ValueError:
            value = self.vars[name].get()
        if not math.isfinite(value):
            value = self.vars[name].get()
        self._updating = True
        scale.configure(from_=min(float(scale.cget("from")), value),
                        to=max(float(scale.cget("to")), value))
        self.vars[name].set(value)
        entry.delete(0, self.tk.END)
        entry.insert(0, f"{value:.6g}")
        self._updating = False
        self._schedule_update()

    def _entry_edited(self, name: str) -> None:
        """Immediately accept complete numeric text without disturbing partial input."""
        if self._updating:
            return
        text = self.entries[name].get().strip()
        if text in ("", "+", "-", ".", "+.", "-."):
            return
        try:
            value = float(text)
        except ValueError:
            return
        if not math.isfinite(value):
            return
        scale = self.scales[name]
        self._updating = True
        try:
            scale.configure(from_=min(float(scale.cget("from")), value),
                            to=max(float(scale.cget("to")), value))
            self.vars[name].set(value)
        finally:
            self._updating = False
        self._schedule_update()

    def _commit_all_entries(self) -> None:
        """Use every visible valid entry as the authoritative current parameter."""
        for name, entry in self.entries.items():
            try:
                value = float(entry.get().strip())
            except ValueError:
                continue
            if not math.isfinite(value):
                continue
            scale = self.scales[name]
            self._updating = True
            try:
                scale.configure(from_=min(float(scale.cget("from")), value),
                                to=max(float(scale.cget("to")), value))
                self.vars[name].set(value)
            finally:
                self._updating = False

    def _config(self) -> SimulationConfig:
        values = {name: variable.get() for name, variable in self.vars.items()}
        values["mode"] = PIDMode.POSITION if self.mode_var.get() == "position" else PIDMode.INCREMENTAL
        return SimulationConfig(**values)

    @staticmethod
    def _plant_signature(config: SimulationConfig) -> tuple[float, ...]:
        return (config.target, config.process_gain, config.process_tau,
                config.process_delay, config.disturbance)

    def _schedule_update(self) -> None:
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
        self._after_id = self.root.after(60, self._update_now)

    def _init_plot_lines(self) -> None:
        titles = ["闭环响应 / Response", "控制器输出 / Output",
                  "控制偏差 / Error", "PID 分量 / Contributions"]
        for axis, title in zip(self.axes.flat, titles):
            axis.set_title(title, fontsize=10)
            axis.grid(True, alpha=0.35)
            axis.set_xlabel("时间 Time (s)", fontsize=8)
        self.lines = {
            "baseline_actual": self.axes[0, 0].plot([], [], "--", color="#8b95a1", label="基准 Baseline")[0],
            "target": self.axes[0, 0].plot([], [], "--", color="#c2413b", label="设定值 Setpoint")[0],
            "actual": self.axes[0, 0].plot([], [], color="#167d73", lw=2, label="过程值 PV")[0],
            "baseline_output": self.axes[0, 1].plot([], [], "--", color="#8b95a1", label="基准 Baseline")[0],
            "output": self.axes[0, 1].plot([], [], color="#0076a8", label="输出 Output")[0],
            "error": self.axes[1, 0].plot([], [], color="#7c3aed")[0],
            "p": self.axes[1, 1].plot([], [], label="比例 P", color="#0076a8")[0],
            "i": self.axes[1, 1].plot([], [], label="积分 I", color="#d97706")[0],
            "d": self.axes[1, 1].plot([], [], label="微分 D", color="#c2413b")[0],
        }
        self.axes[0, 0].legend(fontsize=8)
        self.axes[0, 1].legend(fontsize=8)
        self.axes[1, 1].legend(fontsize=8, ncol=3)

    def _update_now(self) -> None:
        self._after_id = None
        if self._animation_id is not None:
            self.root.after_cancel(self._animation_id)
            self._animation_id = None
        self.status_var.set("计算中 / Updating...")
        self.root.update_idletasks()
        self.data = simulate(self._config())
        self._render(len(self.data["time"]))
        self.status_var.set("就绪 / Ready")

    def _render(self, count: int) -> None:
        count = max(1, min(count, len(self.data["time"])))
        t = self.data["time"][:count]
        for name in ("target", "actual", "output", "error", "p", "i", "d"):
            self.lines[name].set_data(t, self.data[name][:count])
        if self.baseline:
            self.lines["baseline_actual"].set_data(self.baseline["time"], self.baseline["actual"])
            self.lines["baseline_output"].set_data(self.baseline["time"], self.baseline["output"])
        else:
            self.lines["baseline_actual"].set_data([], [])
            self.lines["baseline_output"].set_data([], [])
        for axis in self.axes.flat:
            axis.relim()
            axis.autoscale_view()
        metrics = response_metrics({key: values[:count] for key, values in self.data.items()},
                                   self._config().target)
        self.metric_var.set(
            f"上升 Rise {LegacyPIDSimulator._metric_value(metrics['rise_time'], ' s')}    "
            f"调节 Settling {LegacyPIDSimulator._metric_value(metrics['settling_time'], ' s')}    "
            f"超调 Overshoot {LegacyPIDSimulator._metric_value(metrics['overshoot'], ' %')}    "
            f"稳态偏差 SS Error {LegacyPIDSimulator._metric_value(metrics['steady_error'])}"
        )
        self.canvas.draw_idle()

    def _auto_tune(self) -> None:
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
            self._after_id = None
        self._commit_all_entries()
        if self._animation_id is not None:
            self.root.after_cancel(self._animation_id)
            self._animation_id = None
        self.status_var.set("自动设计与反馈优化中 / Auto designing...")
        self.root.update_idletasks()
        current = self._config()
        signature = self._plant_signature(current)
        if signature != self._tuning_signature:
            initial, refined, report = auto_design(current)
            self.baseline = simulate(initial)
            self._tuning_generation = 1
            self._tuning_signature = signature
        else:
            # The visible fields are authoritative, even if the debounced plot
            # update has not run before the user clicks Auto Tune.
            self.baseline = simulate(current)
            self._tuning_generation += 1
            refined, report = refine_design(current, self._tuning_generation)
        self._updating = True
        tuned_fields = (
            "kp", "ki", "kd", "sample_time", "output_limit", "rate_limit",
            "integral_limit", "deadband", "integral_threshold", "derivative_filter",
            "duration",
        )
        for name in tuned_fields:
            value = getattr(refined, name)
            scale, entry = self.scales[name], self.entries[name]
            scale.configure(from_=min(float(scale.cget("from")), value),
                            to=max(float(scale.cget("to")), value))
            self.vars[name].set(value)
            entry.delete(0, self.tk.END)
            entry.insert(0, f"{value:.6g}")
        self.mode_var.set("position")
        self._updating = False
        self.data = simulate(refined)
        self.status_var.set(
            f"第 {self._tuning_generation} 轮反馈优化 / Generation {self._tuning_generation}: "
            f"{int(report['evaluations'])} 次仿真, "
            f"评分 {report['initial_score']:.3f} -> {report['final_score']:.3f}"
        )
        self._animate(0)

    def _validate_draw_seconds(self, _event: object = None) -> float:
        try:
            value = float(self.draw_seconds.get().strip())
        except (ValueError, self.tk.TclError):
            value = self._valid_draw_seconds
        if not math.isfinite(value) or value <= 0.0:
            value = self._valid_draw_seconds
        value = min(value, 120.0)
        self._valid_draw_seconds = value
        self.draw_seconds.set(f"{value:g}")
        return value

    def _animate(self, frame: int) -> None:
        frames = 24
        count = min(len(self.data["time"]), max(1, math.ceil(len(self.data["time"]) * frame / frames)))
        self._render(count)
        if frame < frames:
            delay = max(30, int(max(0.1, self._validate_draw_seconds()) * 1000 / frames))
            self._animation_id = self.root.after(delay, self._animate, frame + 1)
        else:
            self._animation_id = None

    def _store_baseline(self) -> None:
        self.baseline = {key: values.copy() for key, values in self.data.items()}
        self._render(len(self.data["time"]))

    def _reset(self) -> None:
        self._updating = True
        for name, variable in self.vars.items():
            value = getattr(self.DEFAULTS, name)
            variable.set(value)
            self.entries[name].delete(0, self.tk.END)
            self.entries[name].insert(0, f"{value:g}")
        self.mode_var.set("position")
        self.baseline = None
        self._tuning_signature = None
        self._tuning_generation = 0
        self._updating = False
        self._update_now()

    def _export(self) -> None:
        path = Path.cwd() / "pid_simulation_result.csv"
        with path.open("w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(self.data.keys())
            writer.writerows(zip(*self.data.values()))
        self.status_var.set(f"已导出 / Exported: {path.name}")

    def _close(self) -> None:
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
        if self._animation_id is not None:
            self.root.after_cancel(self._animation_id)
        self.root.destroy()

    def show(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive PID process simulator")
    parser.add_argument("--no-gui", action="store_true", help="run defaults and print final values")
    args = parser.parse_args()
    if args.no_gui:
        data = simulate(SimulationConfig())
        print(f"samples={len(data['time'])} final={data['actual'][-1]:.6f} "
              f"error={data['error'][-1]:.6f} output={data['output'][-1]:.6f}")
    else:
        PIDSimulator().show()


if __name__ == "__main__":
    main()
