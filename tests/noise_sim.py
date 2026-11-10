"""Compare PWM and slow DShot reception under shared Gaussian white noise.

Requires numpy, scipy, matplotlib. See docs/noise-simulation.md.
All times used for scheduling and receiver sampling are integer nanoseconds.
"""
from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import secrets
from copy import deepcopy

import numpy as np
from scipy.signal import lfilter

from motor_noise import (load_profile, validate_profile, generate_motor_noise, SIMULATION_KEYS)
from waveform_html import export_viewer

ROOT = Path(__file__).resolve().parents[1]
IDLE_NS = 1_000_000
WORD = 0xAAAA
METHODS = ("PWM50", "PWM333", "DSHOT4", "DSHOT8", "DSHOT16")
LIMITATIONS = [
    "GPIO thresholds are estimates, not measured STM32 switching thresholds.",
    "Fixed-RMS independent noise changes bandwidth when the time step changes.",
    "No voltage clamps, electrical damage, correlated interference, ISR delays, PWM ramping, or shutdown are modeled.",
    "DShot uses perfect corruption rejection; real four-bit CRCs can miss some corruptions.",
    "Overdue-update and false-command durations may overlap; they must not be added.",
]


@dataclass
class Settings:
    duration_s: float = 2.0
    step_us: float = 1.0
    noise_rms_v: float = 1.0
    vih_v: float = 1.75
    vil_v: float = 1.55
    filter_tau_us: float = 5.0
    pwm_tolerance_us: float = 10.0
    seed: int = 0

    @property
    def step_ns(self):
        return round(self.step_us * 1000)

    @property
    def duration_ns(self):
        return round(self.duration_s * 1e9)

    def validate(self):
        for key, value in asdict(self).items():
            if key != "seed" and not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
        if self.duration_ns <= IDLE_NS or self.step_us <= 0:
            raise ValueError("Duration must exceed the 1 ms startup idle; step must be positive")
        if not math.isclose(self.step_us * 10, round(self.step_us * 10), abs_tol=1e-9):
            raise ValueError("Time step must be a multiple of 0.1 us (100 ns)")
        if min(self.noise_rms_v, self.filter_tau_us, self.pwm_tolerance_us) < 0:
            raise ValueError("Noise RMS, filter time constant, and tolerance cannot be negative")
        if not 0 <= self.vil_v < self.vih_v <= 3.3:
            raise ValueError("Thresholds must satisfy 0 <= V_IL < V_IH <= 3.3 V")
        if self.seed < 0:
            raise ValueError("Seed cannot be negative")
        if self.duration_ns // self.step_ns < 2:
            raise ValueError("Simulation requires at least two time steps")


def generate_noise(count, rms, rng):
    if rms == 0:
        return np.zeros(count)
    noise = rng.normal(size=count)
    noise -= noise.mean()
    measured = np.sqrt(np.mean(noise * noise))
    if measured == 0:
        raise ValueError("Unable to normalize noise")
    return noise * (rms / measured)


def low_pass(values, step_us, tau_us):
    if tau_us == 0:
        return values.copy()
    alpha = -math.expm1(-step_us / tau_us)
    return lfilter([alpha], [1, -(1 - alpha)], values)


def schmitt(values, vih, vil):
    # Forward-fill the last decisive threshold crossing, initially low.
    decisive = (values >= vih) | (values <= vil)
    last = np.maximum.accumulate(np.where(decisive, np.arange(len(values)), -1))
    return (last >= 0) & (values[np.maximum(last, 0)] >= vih)


def waveform(method, times_ns, duration_ns):
    clean = np.zeros(len(times_ns))
    if method.startswith("PWM"):
        frequency = int(method[3:])
        # Round each absolute edge, rather than accumulating rounded periods.
        count = max(0, math.ceil((duration_ns - IDLE_NS) * frequency / 1e9))
        starts = IDLE_NS + np.rint(np.arange(count) * (1e9 / frequency)).astype(np.int64)
        ends = starts + 1_500_000
        for start, end in zip(starts, ends):
            clean[np.searchsorted(times_ns, start):np.searchsorted(times_ns, end)] = 3.3
        return clean, [(int(s), int(e)) for s, e in zip(starts, ends) if e < duration_ns], None
    rate = int(method[5:]) * 5000
    tick_ns = 1_000_000_000 // rate
    bit_ns, frame_ns = 5 * tick_ns, 85 * tick_ns
    starts = np.arange(IDLE_NS, duration_ns, frame_ns, dtype=np.int64)
    relative = times_ns - IDLE_NS
    active = relative >= 0
    position = relative % frame_ns
    bit = position // bit_ns
    safe_bit = np.minimum(bit, 15)
    ones = ((WORD >> (15 - safe_bit)) & 1).astype(bool)
    # All current RMT widths are exact integer nanoseconds.
    high_ns = np.where(ones, bit_ns * 7 // 10, bit_ns * 3 // 10)
    clean[active & (bit < 16) & (position % bit_ns < high_ns)] = 3.3
    expected = [(int(s), int(s + 16 * bit_ns)) for s in starts if s + 16 * bit_ns < duration_ns]
    return clean, expected, tick_ns


def captured_pulses(binary, step_ns):
    transitions = np.diff(binary.astype(np.int8), prepend=0)
    rises = np.flatnonzero(transitions == 1) * step_ns
    falls = np.flatnonzero(transitions == -1) * step_ns
    # A trailing high is incomplete, not a falling edge at capture end.
    return [(int(s), int(e)) for s, e in zip(rises, falls)]


def merged_intervals(intervals):
    episodes = []
    for start, end in sorted(intervals):
        if end <= start:
            continue
        if episodes and start <= episodes[-1][1]:
            episodes[-1] = (episodes[-1][0], max(episodes[-1][1], end))
        else:
            episodes.append((start, end))
    return episodes


def union_duration(intervals):
    return sum(end - start for start, end in merged_intervals(intervals)) / 1e9


def overdue_intervals(deadlines, missing, accepted_times, duration_ns):
    accepted_times = sorted(accepted_times)
    intervals = []
    for frame in sorted(missing):
        deadline = deadlines[frame]
        index = bisect.bisect_left(accepted_times, deadline)
        end = accepted_times[index] if index < len(accepted_times) else duration_ns
        intervals.append((deadline, end))
    return intervals


def base_report(method, expected, matched, rejected_candidates, overdue, false_time=0):
    loss_episodes = merged_intervals(overdue)
    loss_duration = sum(end - start for start, end in loss_episodes) / 1e9
    return dict(method=method, expected_updates=expected, received_updates=matched,
                failed_updates=expected - matched, rejected_candidates=rejected_candidates,
                failure_fraction=(expected - matched) / expected if expected else 0,
                overdue_update_s=loss_duration, signal_loss_episodes=len(loss_episodes),
                average_signal_loss_s=loss_duration / len(loss_episodes) if loss_episodes else 0,
                false_command_s=false_time)


def analyze_pwm(binary, reference, expected, step_ns, tolerance_ns, duration_ns):
    events, matches, incorrect_intervals = [], set(), []
    deadlines = [end for _, end in reference]
    accepted_times = []
    false_start = None
    changed = false_pulses = unmatched = rejected = 0
    expected_ends = [end for _, end in expected]
    for start, end in captured_pulses(binary, step_ns):
        width = end - start
        event = dict(start_ns=start, time_ns=end, width_us=width / 1000)
        if not 800_000 < width < 2_200_000:
            rejected += 1
            events.append(dict(event, status="rejected", reason="PWM width outside acceptance range"))
            continue
        # Maximum overlap among unmatched expected pulses. Pulses are disjoint.
        first = bisect.bisect_right(expected_ends, start)
        choices = []
        for frame in range(first, len(expected)):
            s, e = expected[frame]
            if s >= end:
                break
            if frame not in matches:
                choices.append((min(end, e) - max(start, s), frame))
        frame = max(choices)[1] if choices else None
        if frame is not None:
            matches.add(frame)
            error = width - (reference[frame][1] - reference[frame][0])
            changed += int(error != 0)
            incorrect = abs(error) > tolerance_ns
        else:
            error, incorrect = None, True
            unmatched += 1
        false_pulses += int(incorrect)
        accepted_times.append(end)
        events.append(dict(event, frame_id=frame, status="accepted_false" if incorrect else "accepted_correct",
                           width_error_us=error / 1000 if error is not None else None))
        if incorrect and false_start is None:
            false_start = end
            events.append(dict(time_ns=end, status="command_false"))
        elif not incorrect and false_start is not None:
            incorrect_intervals.append((false_start, end))
            false_start = None
            events.append(dict(time_ns=end, status="command_correct"))
    if false_start is not None:
        incorrect_intervals.append((false_start, duration_ns))
    missing = set(range(len(expected))) - matches
    overdue = overdue_intervals(deadlines, missing, accepted_times, duration_ns)
    for frame in sorted(missing):
        events.append(dict(time_ns=deadlines[frame], frame_id=frame, status="expected_update_missing"))
    report = base_report("", len(expected), len(matches), rejected, overdue, union_duration(incorrect_intervals))
    report.update(accepted_pulses=len(accepted_times), unmatched_accepted_pulses=unmatched,
                  width_changed_pulses=changed, false_command_pulses=false_pulses)
    return report, events, overdue, incorrect_intervals


class DshotSampler:
    """Port of slowDshotSamplePin; check(word, start, end) replaces CRC.

    check returns a frame ID for an accepted word, or None for rejection.
    The simulator's integrity oracle deliberately rejects CRC collisions too.
    """
    def __init__(self, check):
        self.check = check
        self.state = "idle"
        self.low = self.previous = self.index = self.highs = self.bit = 0
        self.saw_low = False
        self.word = 0
        self.start = 0

    def begin_bit(self):
        self.state = "receive"
        self.index = self.highs = 1
        self.saw_low = False

    def sample(self, high, time_ns):
        high = bool(high)
        self.low = 0 if high else min(self.low + 1, 5)
        event = None
        reason = None
        if self.state == "idle":
            if self.low == 5:
                self.state = "edge"
        elif self.state == "edge":
            if high and not self.previous:
                self.bit = self.word = 0
                self.start = time_ns
                self.begin_bit()
        elif self.state == "next":
            if high:
                self.begin_bit()
            elif self.index == 0:
                self.index = 1
            else:
                reason = "second extra low sample"
        else:
            if high and self.saw_low:
                reason = "noncontiguous high prefix"
            else:
                if high:
                    self.highs += 1
                else:
                    self.saw_low = True
                self.index += 1
                if self.index == 5:
                    if high or not 1 <= self.highs <= 4:
                        reason = "invalid fifth sample or high count"
                    else:
                        self.word = (self.word << 1) | int(self.highs >= 3)
                        self.bit += 1
                        self.index = 0
                        self.state = "next"
                        if self.bit == 16:
                            frame = self.check(self.word, self.start, time_ns)
                            if frame is None:
                                reason = "integrity check failed"
                            else:
                                event = dict(start_ns=self.start, time_ns=time_ns, frame_id=frame,
                                             word=f"0x{self.word:04X}", status="accepted_correct")
                                self.state = "edge"
        if reason:
            event = dict(start_ns=self.start, time_ns=time_ns, status="rejected", reason=reason)
            self.state = "idle"
        self.previous = high
        return event


def receive_dshot(binary, step_ns, duration_ns, tick_ns, phase_ns, check):
    sampler = DshotSampler(check)
    events = []
    for time in range(phase_ns, duration_ns, tick_ns):
        index = time // step_ns
        if index >= len(binary):
            break
        event = sampler.sample(binary[index], time)
        if event is not None:
            events.append(event)
    return events


def analyze_dshot(binary, clean_binary, expected, settings, tick_ns, phase_ns):
    starts = [s for s, _ in expected]
    accepted = set()

    def oracle(word, start, end):
        frame = bisect.bisect_right(starts, start) - 1
        if frame < 0 or word != WORD or frame in accepted:
            return None
        s, e = expected[frame]
        if not s <= start < s + tick_ns * 5 or end >= e + tick_ns * 2:
            return None
        accepted.add(frame)
        return frame

    baseline = receive_dshot(clean_binary, settings.step_ns, settings.duration_ns, tick_ns, phase_ns, oracle)
    baseline_good = {e["frame_id"]: e["time_ns"] for e in baseline if e["status"] == "accepted_correct"}
    if len(baseline_good) != len(expected) or any(e["status"] == "rejected" for e in baseline
                                                   if e["start_ns"] < (expected[-1][1] if expected else IDLE_NS)):
        raise ValueError(f"Noise-free DShot baseline failed at {1e9 / tick_ns:g} Hz sampling, "
                         f"phase {phase_ns / 1000:g} us: try --step-us 0.1, reduce filter time constant, "
                         "or adjust thresholds")
    accepted.clear()
    events = receive_dshot(binary, settings.step_ns, settings.duration_ns, tick_ns, phase_ns, oracle)
    # Ignore candidates wholly outside complete transmitted frames (including the trailing fragment).
    last_end = expected[-1][1] + tick_ns * 2 if expected else IDLE_NS
    events = [e for e in events if IDLE_NS <= e["start_ns"] < last_end]
    received = {e["frame_id"] for e in events if e["status"] == "accepted_correct"}
    deadlines = [baseline_good[i] for i in range(len(expected))]
    missing = set(range(len(expected))) - received
    overdue = overdue_intervals(deadlines, missing,
                                [e["time_ns"] for e in events if e["status"] == "accepted_correct"], settings.duration_ns)
    for frame in sorted(missing):
        events.append(dict(time_ns=deadlines[frame], frame_id=frame, status="expected_update_missing"))
    report = base_report("", len(expected), len(received), sum(e["status"] == "rejected" for e in events), overdue)
    report.update(receiver_sample_hz=1e9 / tick_ns, receiver_phase_us=phase_ns / 1000,
                  corrupted_packets=len(missing), false_command_pulses=0)
    return report, events, overdue, []


def waveform_window(events):
    errors = [event["time_ns"] for event in events
              if event["status"] in ("rejected", "accepted_false", "expected_update_missing")
              or event.get("width_error_us", 0) not in (None, 0)]
    first = min(errors) if errors else None
    return (max(0, first - 2_000_000) if first is not None else 0), first


def waveform_preview(times, clean, noisy, filtered, binary, start_ns, step_ns, overdue, false):
    section = slice(start_ns // step_ns, int(np.searchsorted(times, start_ns + 25_000_000)))
    return dict(time_ms=times[section] / 1e6, clean=clean[section].copy(),
                noisy=noisy[section].copy(), filtered=filtered[section].copy(),
                binary=binary[section].copy(), overdue=overdue, false=false,
                window_start_ns=start_ns)


def simulate(settings, save_waveforms=False, profile=None):
    settings.validate()
    if profile is not None:
        profile = deepcopy(profile)
        profile["background_rms_v"] = settings.noise_rms_v
        profile["simulation"] = {key: getattr(settings, key) for key in SIMULATION_KEYS}
        validate_profile(profile, settings.step_us)
    times = np.arange(0, settings.duration_ns, settings.step_ns, dtype=np.int64)
    noise_seed, phase_seed = np.random.SeedSequence(settings.seed).spawn(2)
    noise = generate_noise(len(times), settings.noise_rms_v, np.random.default_rng(noise_seed))
    components, disturbance_plot, profile_info = {}, None, None
    if profile is not None:
        noise, components, disturbance_plot, profile_info = generate_motor_noise(
            times, settings.step_ns, profile, settings.seed, noise)
    # A shared fractional phase maps to each receiver's physical sampling period.
    phase_fraction = float(np.random.default_rng(phase_seed).random())
    reports, events, plots = [], [], {}
    arrays = {"time_ns": times, "noise_v": noise} if save_waveforms else {}
    if save_waveforms:
        arrays.update(components)
    # Without full export, free large component arrays after taking previews.
    del components
    for method in METHODS:
        clean, expected, tick = waveform(method, times, settings.duration_ns)
        if not expected:
            raise ValueError(f"Duration is too short for a complete {method} transmission")
        noisy = clean + noise
        filtered = low_pass(noisy, settings.step_us, settings.filter_tau_us)
        binary = schmitt(filtered, settings.vih_v, settings.vil_v)
        baseline_binary = schmitt(low_pass(clean, settings.step_us, settings.filter_tau_us), settings.vih_v, settings.vil_v)
        if tick is None:
            baseline_pulses = captured_pulses(baseline_binary, settings.step_ns)
            expected_ends = [e for _, e in expected]
            reference = []
            for pulse in baseline_pulses:
                frame = bisect.bisect_right(expected_ends, pulse[0])
                if frame < len(expected) and expected[frame][0] < pulse[1]:
                    reference.append(pulse)
            if len(reference) != len(expected) or any(not 800_000 < e - s < 2_200_000 for s, e in reference):
                raise ValueError("Noise-free PWM baseline failed: adjust resolution/filter/thresholds")
            result = analyze_pwm(binary, reference, expected, settings.step_ns,
                                 settings.pwm_tolerance_us * 1000, settings.duration_ns)
        else:
            result = analyze_dshot(binary, baseline_binary, expected, settings, tick, int(phase_fraction * tick))
        report, method_events, overdue, false = result
        report["method"] = method
        reports.append(report)
        events.extend(dict(e, method=method) for e in method_events)
        # Keep only a short view per method, not all full-resolution waveforms.
        start_ns, _ = waveform_window(method_events)
        plots[method] = waveform_preview(times, clean, noisy, filtered, binary,
                                         start_ns, settings.step_ns, overdue, false)
        if save_waveforms:
            arrays.update({f"{method}_clean_v": clean, f"{method}_noisy_v": noisy,
                           f"{method}_filtered_v": filtered, f"{method}_binary": binary})
    # Align every panel to the earliest error across all methods. Regenerate
    # only previews whose local window differed; filtering retains full history.
    start_ns, first_error_ns = waveform_window(events)
    for method in METHODS:
        p = plots[method]
        if p["window_start_ns"] == start_ns:
            continue
        if save_waveforms:
            clean, noisy, filtered, binary = (arrays[f"{method}_{key}"] for key in
                                              ("clean_v", "noisy_v", "filtered_v", "binary"))
        else:
            clean, _, _ = waveform(method, times, settings.duration_ns)
            noisy = clean + noise
            filtered = low_pass(noisy, settings.step_us, settings.filter_tau_us)
            binary = schmitt(filtered, settings.vih_v, settings.vil_v)
        plots[method] = waveform_preview(times, clean, noisy, filtered, binary,
                                         start_ns, settings.step_ns, p["overdue"], p["false"])
    summary = dict(settings=asdict(settings), measured_noise_rms_v=float(np.sqrt(np.mean(noise * noise))),
                   measured_noise_mean_v=float(noise.mean()), limitations=list(LIMITATIONS), reports=reports,
                   waveform_window_start_s=start_ns / 1e9,
                   first_signal_error_s=first_error_ns / 1e9 if first_error_ns is not None else None)
    if profile is not None:
        summary.update(effective_profile=profile, disturbance=profile_info)
        summary["limitations"][2] = "No voltage clamps, electrical damage, ISR delays, PWM ramping, or shutdown are modeled."
        summary["limitations"].append("Motor profiles are illustrative GPIO-referred stress cases, not calibrated robot predictions.")
        plots["disturbance"] = disturbance_plot
    return summary, events, plots, arrays


def write_csv(path, rows):
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def save_outputs(output, stamp, summary, events, plots, arrays):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not arrays:
        raise ValueError("Full waveforms are required for binary and interactive export; call simulate(..., True)")
    output = output / stamp
    output.mkdir(parents=True, exist_ok=False)
    prefix = output / stamp
    summary["started_utc"] = stamp
    prefix.with_name(stamp + "_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_csv(prefix.with_name(stamp + "_summary.csv"),
              [dict(summary["settings"], measured_noise_rms_v=summary["measured_noise_rms_v"],
                    effective_profile_json=json.dumps(summary.get("effective_profile")),
                    disturbance_measurements_json=json.dumps(summary.get("disturbance")), **r) for r in summary["reports"]])
    converted = []
    for event in sorted(events, key=lambda e: (e["time_ns"], e["method"])):
        converted.append({(k[:-3] + "_s" if k.endswith("_ns") else k):
                          (v / 1e9 if k.endswith("_ns") else v) for k, v in event.items()})
    write_csv(prefix.with_name(stamp + "_events.csv"), converted)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    axes[0].bar(METHODS, [r["failure_fraction"] * 100 for r in summary["reports"]])
    axes[0].set_ylabel("Expected updates missed (%)")
    axes[1].bar(np.arange(5) - .2, [r["overdue_update_s"] for r in summary["reports"]], .4, label="Overdue update")
    axes[1].bar(np.arange(5) + .2, [r["false_command_s"] for r in summary["reports"]], .4, label="False command")
    axes[1].set_xticks(np.arange(5), METHODS)
    axes[1].set_ylabel("Affected duration (s)")
    axes[1].legend()
    title = summary.get("effective_profile", {}).get("name", "white noise")
    fig.suptitle(f"Shared {title}; DShot integrity checking is idealized")
    fig.tight_layout()
    fig.savefig(prefix.with_name(stamp + "_comparison.png"), dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(5, 1, figsize=(13, 12), sharex=True)
    for ax, method in zip(axes, METHODS):
        p = plots[method]
        ax.plot(p["time_ms"], p["noisy"], color="gray", alpha=.35, linewidth=.4, label="Noisy voltage")
        ax.plot(p["time_ms"], p["clean"], linewidth=.8, label="Clean voltage")
        ax.plot(p["time_ms"], p["filtered"], linewidth=.8, label="Filtered voltage")
        ax.step(p["time_ms"], p["binary"] * 3.3, where="post", linewidth=.6, label="GPIO × 3.3 V")
        ax.set_ylabel(method + "\nVoltage (V)")
        ax.axhline(summary["settings"]["vih_v"], color="black", linestyle=":", linewidth=.5)
        ax.axhline(summary["settings"]["vil_v"], color="black", linestyle=":", linewidth=.5)
        if summary.get("first_signal_error_s") is not None:
            ax.axvline(summary["first_signal_error_s"] * 1000, color="red", linestyle="--", linewidth=.7)
    axes[0].legend(ncol=4, fontsize=8)
    start_s = summary.get("waveform_window_start_s", 0)
    end_s = min(summary["settings"]["duration_s"], start_s + .025)
    axes[-1].set_xlim(start_s * 1000, end_s * 1000)
    axes[-1].set_xlabel("Time (ms); 25 ms window, starting 2 ms before first signal error (or zero)")
    fig.tight_layout()
    fig.savefig(prefix.with_name(stamp + "_waveforms.png"), dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(5, 1, figsize=(12, 7), sharex=True)
    for ax, method in zip(axes, METHODS):
        for key, level, color in (("overdue", .6, "orange"), ("false", .1, "red")):
            spans = [(s / 1e9, (e - s) / 1e9) for s, e in plots[method][key]]
            ax.broken_barh(spans, (level, .3), facecolors=color)
        ax.set_ylabel(method)
        ax.set_ylim(0, 1)
        ax.set_yticks([])
    axes[0].set_title("Held-command timeline: orange = overdue update; red = false command")
    axes[-1].set_xlim(0, summary["settings"]["duration_s"])
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(prefix.with_name(stamp + "_timeline.png"), dpi=150)
    plt.close(fig)
    if "disturbance" in plots:
        p = plots["disturbance"]
        fig, axes = plt.subplots(3, 1, figsize=(12, 9))
        axes[0].plot(p["time_s"], p["startup_envelope"], label="Startup multiplier")
        axes[0].fill_between(p["time_s"], p["electrical_modulation_min"], p["electrical_modulation_max"],
                             color="orange", alpha=.4, label="Electrical-cycle multiplier range per display interval")
        axes[0].set_ylabel("Amplitude multiplier")
        axes[0].set_xlabel("Time (s)")
        axes[1].plot(p["time_s"], p["ground_v"], label="Receiver ground displacement (subtracted)")
        axes[1].set_ylabel("Ground shift (V)")
        axes[1].set_xlabel("Time (s)")
        axes[2].plot(p["close_time_us"], p["close_spikes_v"], label="Spikes")
        axes[2].plot(p["close_time_us"], p["close_ringing_v"], label="Ringing")
        axes[2].plot(p["close_time_us"], p["close_combined_v"], alpha=.6, label="Combined disturbance")
        axes[2].set_xlabel("Time since motor startup (us)")
        axes[2].set_ylabel("Disturbance (V)")
        for ax in axes:
            ax.legend(fontsize=8)
            ax.grid(alpha=.2)
        fig.suptitle(f"{title}: GPIO-referred disturbance, before receiver filtering")
        fig.tight_layout()
        fig.savefig(prefix.with_name(stamp + "_disturbance.png"), dpi=150)
        plt.close(fig)
    np.savez_compressed(prefix.with_name(stamp + "_waveforms.npz"), **arrays)
    export_viewer(prefix.with_name(stamp + "_waveforms.html"), summary, arrays)
    return output


def parse_options(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    for option, default, help_text in (
            ("duration-s", 2., "Total simulation duration in seconds"),
            ("step-us", 1., "Waveform resolution in microseconds; multiples of 0.1"),
            ("noise-rms-v", 1., "Measured RMS of shared zero-mean Gaussian noise, volts"),
            ("vih-v", 1.75, "Estimated GPIO rising threshold, volts"),
            ("vil-v", 1.55, "Estimated GPIO falling threshold, volts"),
            ("filter-tau-us", 5., "Low-pass time constant, microseconds; 0 disables filtering"),
            ("pwm-tolerance-us", 10., "Width error above which a matched PWM pulse is a false command")):
        parser.add_argument("--" + option, type=float, default=argparse.SUPPRESS,
                            help=help_text + f"; no-profile default {default:g}")
    parser.add_argument("--seed", type=int, default=None, help="Random seed; generated and recorded if omitted")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "obj/noise_sim")
    parser.add_argument("--save-waveforms", action="store_true", help="Compatibility flag; full binary waveforms are now always saved")
    parser.add_argument("--profile", help="Bundled profile name or path to custom JSON")
    args = parser.parse_args(argv)
    try:
        profile = load_profile(args.profile) if args.profile else None
        values = asdict(Settings())
        if profile is not None:
            values.update(profile["simulation"])
            values["noise_rms_v"] = profile["background_rms_v"]
        values.update({k: v for k, v in vars(args).items() if k in values and v is not None})
        settings = Settings(**values)
        if args.seed is None:
            settings.seed = secrets.randbits(64)
        settings.validate()
        if profile is not None:
            profile["background_rms_v"] = settings.noise_rms_v
            validate_profile(profile, settings.step_us)
    except ValueError as error:
        parser.error(str(error))
    return parser, args, settings, profile


def main():
    started = datetime.now(timezone.utc)
    parser, args, settings, profile = parse_options()
    if settings.seed is None:
        settings.seed = secrets.randbits(64)
    print(f"Started {started.isoformat()}, seed {settings.seed}", flush=True)
    try:
        summary, events, plots, arrays = simulate(settings, True, profile)
    except ValueError as error:
        parser.error(str(error))
    stamp = started.strftime("%Y%m%dT%H%M%S_%fZ")
    run_dir = save_outputs(args.output_dir, stamp, summary, events, plots, arrays)
    print(f"{'Method':<10} {'Expected':>9} {'Received':>9} {'Failed':>9} {'Overdue(s)':>12} {'Avg loss(ms)':>13} {'False(s)':>12}")
    for row in summary["reports"]:
        print(f"{row['method']:<10} {row['expected_updates']:>9} {row['received_updates']:>9} "
              f"{row['failed_updates']:>9} {row['overdue_update_s']:>12.6f} "
              f"{row['average_signal_loss_s'] * 1000:>13.6f} {row['false_command_s']:>12.6f}")
    print(f"Reports and binary waveforms: {run_dir}")
    print(f"Interactive viewer: {run_dir / (stamp + '_waveforms.html')}")
    for limitation in summary["limitations"]:
        print("Model: " + limitation)


if __name__ == "__main__":
    main()
