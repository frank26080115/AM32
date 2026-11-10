"""Profile loading and GPIO-referred motor interference; no motor solver."""
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np

PROFILE_DIR = Path(__file__).with_name("noise_profiles")
DEFAULT_PROFILE = {
    "background_rms_v": 0.1,
    "startup": {"at_s": .25, "peak_multiplier": 4., "decay_ms": 50.},
    "motor": {"electrical_hz": 1000., "spinup_s": .5, "modulation_depth": .5},
    "switching": {"pwm_hz": 24000., "duty": .5, "spike_peak_v": .5,
                  "spike_width_us": 1., "ring_peak_v": .25,
                  "ring_hz": 200000., "ring_decay_us": 3.},
    "ground_shift": {"startup_v": .8, "steady_v": .05, "decay_ms": 20.},
    "simulation": {"step_us": .1},
}
SIMULATION_KEYS = {"duration_s", "step_us", "vih_v", "vil_v", "filter_tau_us", "pwm_tolerance_us"}


def load_profile(name):
    path = Path(name)
    if not path.is_file():
        path = PROFILE_DIR / (str(name) + ".json")
    try:
        supplied = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Cannot read noise profile {name}: {error}") from error
    if not isinstance(supplied, dict):
        raise ValueError("Noise profile must be a JSON object")
    result = deepcopy(DEFAULT_PROFILE)
    for key, value in supplied.items():
        if key in ("name", "description"):
            if not isinstance(value, str):
                raise ValueError(f"{key} must be text")
            result[key] = value
        elif key not in result:
            raise ValueError(f"Unknown profile field {key}")
        elif isinstance(result[key], dict):
            allowed = SIMULATION_KEYS if key == "simulation" else set(result[key])
            if not isinstance(value, dict) or set(value) - allowed:
                raise ValueError(f"Invalid fields in {key}")
            result[key].update(value)
        else:
            result[key] = value
    result.setdefault("name", path.stem)
    validate_profile(result)
    return result


def validate_profile(profile, step_us=None):
    for group, value in profile.items():
        if group in ("name", "description"):
            continue
        items = value.items() if isinstance(value, dict) else [(group, value)]
        for key, number in items:
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
                raise ValueError(f"{group}.{key} must be a finite number")
            if group != "ground_shift" or key == "decay_ms":
                if number < 0:
                    raise ValueError(f"{group}.{key} cannot be negative")
    sw = profile["switching"]
    if not 0 < sw["duty"] < 1:
        raise ValueError("switching.duty must be strictly between 0 and 1")
    if profile["startup"]["peak_multiplier"] < 1:
        raise ValueError("startup.peak_multiplier must be at least 1")
    if profile["startup"]["decay_ms"] <= 0 or profile["ground_shift"]["decay_ms"] <= 0:
        raise ValueError("Startup and ground decay times must be positive")
    if sw["spike_peak_v"] > 0 and (sw["spike_width_us"] <= 0 or sw["pwm_hz"] <= 0):
        raise ValueError("Active spikes require positive width and PWM frequency")
    if sw["ring_peak_v"] > 0 and min(sw["ring_hz"], sw["ring_decay_us"], sw["pwm_hz"]) <= 0:
        raise ValueError("Active ringing requires positive frequency, decay, and PWM frequency")
    if step_us is not None:
        limits = []
        if sw["spike_peak_v"]:
            limits.append(sw["spike_width_us"] / 4)
        if sw["ring_peak_v"]:
            limits.append(1e6 / (10 * sw["ring_hz"]))
        if (sw["spike_peak_v"] or sw["ring_peak_v"]) and sw["pwm_hz"]:
            limits.append(1e6 * min(sw["duty"], 1 - sw["duty"]) / sw["pwm_hz"])
        if limits and step_us > min(limits) + 1e-12:
            maximum = math.floor((min(limits) + 1e-12) * 10) / 10
            raise ValueError(f"Profile requires step <= {min(limits):g} us "
                             f"(supported maximum {maximum:g} us); use --step-us {maximum:g}" if maximum >= .1
                             else "Profile requires resolution finer than the supported 100 ns minimum")


def startup_envelope(age_s, profile):
    p = profile["startup"]
    return np.where(age_s >= 0, 1 + (p["peak_multiplier"] - 1) *
                    np.exp(-np.maximum(age_s, 0) / (p["decay_ms"] / 1000)), 0)


def electrical_phase(age_s, profile, initial_phase=0):
    p = profile["motor"]
    u = np.maximum(age_s, 0)
    ramp = p["spinup_s"]
    cycles = p["electrical_hz"] * (u if ramp == 0 else
             np.where(u < ramp, u * u / (2 * ramp), u - ramp / 2))
    return initial_phase + 2 * np.pi * cycles


def ground_displacement(age_s, profile):
    p = profile["ground_shift"]
    return np.where(age_s >= 0, p["steady_v"] + (p["startup_v"] - p["steady_v"]) *
                    np.exp(-np.maximum(age_s, 0) / (p["decay_ms"] / 1000)), 0)


def switching_events(duration_ns, profile, phase_fraction):
    p = profile["switching"]
    if p["pwm_hz"] == 0 or not (p["spike_peak_v"] or p["ring_peak_v"]):
        return np.array([], dtype=np.int64), np.array([], dtype=np.int8)
    period = 1e9 / p["pwm_hz"]
    start = profile["startup"]["at_s"] * 1e9 + phase_fraction * period
    cycles = np.arange(max(0, math.ceil((duration_ns - start) / period)))
    rises = start + cycles * period
    times = np.rint(np.column_stack((rises, rises + p["duty"] * period)).ravel()).astype(np.int64)
    signs = np.tile([1, -1], len(cycles)).astype(np.int8)
    mask = times < duration_ns
    return times[mask], signs[mask]


def event_kernel(age_us, profile):
    p = profile["switching"]
    width = p["spike_width_us"]
    spike = np.zeros_like(age_us, dtype=float)
    if p["spike_peak_v"]:
        spike = p["spike_peak_v"] * np.maximum(0, 1 - np.abs(2 * age_us / width - 1))
        spike[age_us < 0] = 0
    # Ring starts at the end of the triangular spike. With spikes disabled it
    # starts at the event itself. ring_peak_v is the exponential envelope peak.
    elapsed = age_us - (width if p["spike_peak_v"] else 0)
    ring = np.zeros_like(age_us, dtype=float)
    if p["ring_peak_v"]:
        active = elapsed >= 0
        ring[active] = p["ring_peak_v"] * np.exp(-elapsed[active] / p["ring_decay_us"]) * np.sin(
            2 * np.pi * p["ring_hz"] * elapsed[active] / 1e6)
    return spike, ring


def measurements(values):
    # Chunk reductions avoid a full-sized temporary square of a waveform.
    energy = total = 0.
    peak = 0.
    for first in range(0, len(values), 100000):
        chunk = values[first:first + 100000].astype(np.float64, copy=False)
        energy += float(np.dot(chunk, chunk))
        total += float(chunk.sum())
        peak = max(peak, float(np.max(np.abs(chunk))))
    return {"rms_v": math.sqrt(energy / len(values)), "peak_abs_v": peak, "mean_v": total / len(values)}


def generate_motor_noise(times_ns, step_ns, profile, seed, background):
    validate_profile(profile, step_ns / 1000)
    streams = np.random.SeedSequence(seed).spawn(4)
    switch_phase = float(np.random.default_rng(streams[2]).random())
    electric_phase = float(np.random.default_rng(streams[3]).uniform(0, 2 * np.pi))
    components = {"background_v": background,
                  "switching_spikes_v": np.zeros(len(times_ns), dtype=np.float32),
                  "switching_ringing_v": np.zeros(len(times_ns), dtype=np.float32),
                  "ground_shift_v": np.zeros(len(times_ns), dtype=np.float32)}
    onset_ns = round(profile["startup"]["at_s"] * 1e9)
    for first in range(0, len(times_ns), 100000):
        age = (times_ns[first:first + 100000] - onset_ns) / 1e9
        components["ground_shift_v"][first:first + len(age)] = ground_displacement(age, profile)
    event_times, signs = switching_events(int(times_ns[-1] + step_ns), profile, switch_phase)
    ages = (event_times - onset_ns) / 1e9
    scales = startup_envelope(ages, profile) * (1 + profile["motor"]["modulation_depth"] *
             np.abs(np.sin(electrical_phase(ages, profile, electric_phase)))) * signs
    sw = profile["switching"]
    # Ring tails are truncated at exp(-12), below 0.000007 of their envelope.
    tail_us = (sw["spike_width_us"] if sw["spike_peak_v"] else 0) + 12 * sw["ring_decay_us"]
    length = math.ceil(tail_us * 1000 / step_ns) + 1
    relative_ns = np.arange(length, dtype=np.int64) * step_ns
    for event, scale in zip(event_times, scales):
        first = (int(event) + step_ns - 1) // step_ns
        end = min(len(times_ns), first + length)
        age_us = (relative_ns[:end - first] + first * step_ns - event) / 1000
        spike, ring = event_kernel(age_us, profile)
        components["switching_spikes_v"][first:end] += spike * scale
        components["switching_ringing_v"][first:end] += ring * scale
    disturbance = components["background_v"].copy()
    disturbance += components["switching_spikes_v"]
    disturbance += components["switching_ringing_v"]
    disturbance -= components["ground_shift_v"]
    # Full-run envelopes are only retained as a compact plot preview.
    indices = np.unique(np.linspace(0, len(times_ns) - 1, min(len(times_ns), 4000)).astype(int))
    age = (times_ns[indices] - onset_ns) / 1e9
    phase_lo = electrical_phase(age, profile, electric_phase)
    phase_hi = electrical_phase(np.r_[age[1:], age[-1]], profile, electric_phase)
    endpoint_lo, endpoint_hi = np.abs(np.sin(phase_lo)), np.abs(np.sin(phase_hi))
    cycle_min = np.minimum(endpoint_lo, endpoint_hi)
    cycle_max = np.maximum(endpoint_lo, endpoint_hi)
    # Exact extrema of |sin| within each display interval prevent aliasing of
    # fast electrical cycles into a fictitious slow modulation curve.
    cycle_min[np.ceil(phase_lo / np.pi) <= np.floor(phase_hi / np.pi)] = 0
    cycle_max[np.ceil((phase_lo - np.pi / 2) / np.pi) <= np.floor((phase_hi - np.pi / 2) / np.pi)] = 1
    depth = profile["motor"]["modulation_depth"]
    close_start = max(0, onset_ns - 10_000)
    close_end = min(int(times_ns[-1] + step_ns), onset_ns + 300_000)
    section = slice(int(np.searchsorted(times_ns, close_start)), int(np.searchsorted(times_ns, close_end)))
    preview = {"time_s": times_ns[indices] / 1e9,
               "startup_envelope": startup_envelope(age, profile),
               "electrical_modulation_min": np.where(age >= 0, 1 + depth * cycle_min, 0),
               "electrical_modulation_max": np.where(age >= 0, 1 + depth * cycle_max, 0),
               "ground_v": components["ground_shift_v"][indices].copy(),
               "close_time_us": (times_ns[section] - onset_ns) / 1000,
               "close_spikes_v": components["switching_spikes_v"][section].copy(),
               "close_ringing_v": components["switching_ringing_v"][section].copy(),
               "close_combined_v": disturbance[section].copy()}
    info = {"switching_phase_fraction": switch_phase, "electrical_initial_phase_rad": electric_phase,
            "switching_event_count": len(event_times), "ring_tail_decay_constants": 12,
            "components": {name: measurements(values) for name, values in components.items()},
            "combined": measurements(disturbance)}
    return disturbance, components, preview, info
