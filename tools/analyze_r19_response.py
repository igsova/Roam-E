#!/usr/bin/env python3
"""Offline per-wheel response review of saved R19 paired-hold records.

No hardware access, serial, motion or battery inference. Reads saved helper
records ({raw, derived}) and reconstructs, on the Nano's own millis clock:
commanded PWM, raw count-delta speed, firmware-filtered speed, RUN phase and
host command/telemetry timing. It then summarizes, per wheel and direction,
how much duty the settled controller needed to hold the target and how that
requirement drifted inside one episode.

Interpretation limits: 50 ms telemetry snapshots can miss 20 ms control
excursions; PWM is the requested duty, not measured motor voltage/current;
encoder counts are not independent physical displacement.

Usage: python3 tools/analyze_r19_response.py [record.json ...]
       (no arguments: every docs/measurements/R19_*.json with paired telemetry)
       python3 tools/analyze_r19_response.py --position
       (V3 required-duty proxy versus chained wheel position)
       python3 tools/analyze_r19_response.py --id record.json ...
       (ID_V1 duty-step records: level speeds, incremental gain, step fit)
"""
from __future__ import annotations

import glob
import json
import math
import statistics
import sys
from pathlib import Path

SETTLE_AFTER_RUN_MS = 500  # Exclude the startup handoff transient.
STOP_MARGIN_MS = 50  # Exclude the last sample before host STOP.

# ATmega328P/Arduino AVR core: D5/D6 -> timer0 (fast PWM, ~977 Hz),
# D9/D10 -> timer1 (8-bit phase-correct, ~490 Hz). Which pin carries PWM
# follows setDrv8871Output() and the R19 polarity (left +1, right -1).
PWM_PIN = {
    ("left", 1): ("D5", "timer0 ~977Hz fast"),
    ("left", -1): ("D10", "timer1 ~490Hz phase-correct"),
    ("right", 1): ("D9", "timer1 ~490Hz phase-correct"),
    ("right", -1): ("D6", "timer0 ~977Hz fast"),
}


def counter_delta(a: int, b: int) -> int:
    return ((a - b + 2**31) % 2**32) - 2**31


def load(path: str) -> dict | None:
    try:
        record = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None
    raw = record.get("raw") if isinstance(record, dict) else None
    if not isinstance(raw, dict) or "telemetry" not in raw or "targets" not in raw:
        return None
    return raw


def load_any(path: str) -> dict:
    record = json.loads(Path(path).read_text())
    return record.get("raw", record)


def rows_for(raw: dict) -> list[dict]:
    """Velocity-phase telemetry joined with bounded phase lines by millis."""
    phase_by_ms = {
        p["values"][0]: p["values"][1:]
        for p in raw.get("phases", [])
        if p.get("phase") in ("velocity", "motion")  # V3 / ID_V1 helper names
    }
    out = []
    previous = None
    for t in raw["telemetry"]:
        if t.get("phase") not in ("velocity", "motion"):
            continue
        v = t["values"]
        row = {
            "ms": v[0],
            "host_s": t["elapsed_s"],
            "ticks": (v[1], v[2]),
            "pwm": (v[3], v[4]),
            "target": (v[5], v[6]),
            "filtered": (v[7], v[8]),
            "bounded": phase_by_ms.get(v[0]),
            "raw": None,
            "dt_ms": None,
        }
        if previous is not None:
            dt = counter_delta(v[0], previous["ms"])
            row["dt_ms"] = dt
            if dt > 0:
                row["raw"] = tuple(
                    counter_delta(row["ticks"][i], previous["ticks"][i]) * 1000.0 / dt
                    for i in (0, 1)
                )
        out.append(row)
        previous = row
    return out


def pearson(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 3:
        return None
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    if sxx == 0 or syy == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / math.sqrt(sxx * syy)


def slope(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 3:
        return None
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx


def analyze(raw: dict, name: str) -> dict:
    sign = 1 if raw["targets"][0] > 0 else -1
    rows = rows_for(raw)
    run_rows = [r for r in rows if r["bounded"] and r["bounded"][0] == 2 and r["bounded"][1] == 2]
    result: dict = {
        "record": name,
        "episode": raw.get("episode"),
        "caps": (raw.get("caps") or "").split(",")[1:2],
        "profile": raw.get("profile"),
        "reason": raw.get("reason"),
        "nominal_s": raw.get("nominal_seconds"),
    }
    dts = [r["dt_ms"] for r in rows if r["dt_ms"] is not None]
    result["telemetry_dt_ms"] = (min(dts), max(dts)) if dts else None
    vel_times = [c["elapsed_s"] for c in raw.get("commands", []) if c["command"].startswith("VEL,")]
    gaps = [b - a for a, b in zip(vel_times, vel_times[1:])]
    result["vel_refresh_s"] = (
        (round(min(gaps), 4), round(statistics.fmean(gaps), 4), round(max(gaps), 4)) if gaps else None
    )
    if not run_rows:
        result["no_run"] = True
        return result
    run_ms = run_rows[0]["ms"]
    end_ms = rows[-1]["ms"] - STOP_MARGIN_MS
    settled = [
        r for r in run_rows
        if counter_delta(r["ms"], run_ms) >= SETTLE_AFTER_RUN_MS
        and counter_delta(end_ms, r["ms"]) >= 0
        and r["raw"] is not None
    ]
    result["run_at_ms_after_first"] = counter_delta(run_ms, rows[0]["ms"])
    result["settled_samples"] = len(settled)
    wheels = {}
    for i, side in enumerate(("left", "right")):
        if len(settled) < 12:
            break
        pwm = [sign * r["pwm"][i] for r in settled]
        speed = [sign * r["raw"][i] for r in settled]
        filt = [sign * r["filtered"][i] for r in settled]
        ticks = [sign * counter_delta(r["ticks"][i], settled[0]["ticks"][i]) for r in settled]
        # PWM from the previous sample is what produced this interval's counts.
        lag_pwm = [sign * settled[k - 1]["pwm"][i] for k in range(1, len(settled))]
        lag_speed = speed[1:]
        # Moving 0.5 s (10-sample) averages expose slow drift of required duty.
        win = 10
        avg_pwm = [statistics.fmean(pwm[k:k + win]) for k in range(0, len(pwm) - win + 1)]
        avg_speed = [statistics.fmean(speed[k:k + win]) for k in range(0, len(speed) - win + 1)]
        hi = max(range(len(avg_pwm)), key=avg_pwm.__getitem__)
        lo = min(range(len(avg_pwm)), key=avg_pwm.__getitem__)
        pin, timer = PWM_PIN[(side, sign)]
        if raw.get("timer1_fast") and timer.startswith("timer1"):
            timer = "timer1 ~977Hz fast (PWMF,1)"
        wheels[side] = {
            "pin": pin,
            "timer": timer,
            "pwm_mean": round(statistics.fmean(pwm), 1),
            "pwm_min_max": (min(pwm), max(pwm)),
            "pwm_sd": round(statistics.pstdev(pwm), 1),
            "speed_raw_mean": round(statistics.fmean(speed), 1),
            "speed_raw_sd": round(statistics.pstdev(speed), 1),
            "speed_filtered_mean": round(statistics.fmean(filt), 1),
            "avg0p5s_pwm_range": (round(avg_pwm[lo], 1), round(avg_pwm[hi], 1)),
            "avg0p5s_speed_at_pwm_low_high": (round(avg_speed[lo], 1), round(avg_speed[hi], 1)),
            "ticks_at_pwm_low_high": (ticks[lo + win // 2], ticks[hi + win // 2]),
            "duty_per_200tps": round(statistics.fmean(pwm) * 200 / statistics.fmean(speed), 1),
            # Negative correlation = more duty while slower (load-driven);
            # positive = speed follows duty (controller-driven variation).
            "corr_prev_pwm_vs_speed": round(pearson(lag_pwm, lag_speed) or 0, 3),
            "corr_avg_pwm_vs_avg_speed": round(pearson(avg_pwm, avg_speed) or 0, 3),
        }
    result["wheels"] = wheels
    return result


# Saved V3 episode order on 13 Oct (R19_RELIABILITY_SUMMARY V3_episodes). Each
# record's final encoder_delta (including coast) is chained to place samples on
# a common per-wheel position axis. Assumes no unrecorded motion between
# episodes; straight-line wheel angle and floor position stay locked together.
V3_ORDER = [
    "FORWARD_SMOKE", "REVERSE_SMOKE", "FORWARD_TRACK", "REVERSE_TRACK",
    "FORWARD1", "REVERSE1", "FORWARD2", "REVERSE2", "FORWARD3",
    "DAMPED_REVERSE_SMOKE", "DAMPED_FORWARD_SMOKE", "DAMPED_REVERSE_TRACK",
    "DAMPED_FORWARD_TRACK",
]
LOAD_GAIN_TICKS_PER_PWM = 10.0  # Assumed; profiles were checked at 5 and 20.
BIN_TICKS = 50


def position_profiles(directory: str = "docs/measurements") -> dict:
    """Required-duty proxy versus chained wheel position for V3 holds.

    proxy = 0.5 s mean PWM - (0.5 s mean raw speed - 200) / LOAD_GAIN.
    Each run's mean is removed so forward/reverse offsets do not dominate.
    """
    offset = [0, 0]
    profiles: dict = {"left": {}, "right": {}}
    for name in V3_ORDER:
        raw = load(f"{directory}/R19_HANDOFF_{name}_20261013.json")
        if raw is None:
            raise FileNotFoundError(name)
        sign = 1 if raw["targets"][0] > 0 else -1
        rows = rows_for(raw)
        run = [r for r in rows if r["bounded"] and r["bounded"][:2] == [2, 2]]
        if run and raw["nominal_seconds"] >= 3:
            t0 = run[0]["ms"]
            st = [r for r in run if counter_delta(r["ms"], t0) >= SETTLE_AFTER_RUN_MS and r["raw"]]
            for i, side in enumerate(("left", "right")):
                pts = []
                for k in range(len(st) - 9):
                    w = st[k:k + 10]
                    pwm = statistics.fmean(sign * r["pwm"][i] for r in w)
                    speed = sign * counter_delta(w[-1]["ticks"][i], w[0]["ticks"][i]) * 1000 / counter_delta(w[-1]["ms"], w[0]["ms"])
                    pos = offset[i] + statistics.fmean(r["ticks"][i] for r in w)
                    pts.append((pos, pwm - (speed - 200) / LOAD_GAIN_TICKS_PER_PWM))
                mean = statistics.fmean(p[1] for p in pts)
                bins: dict = {}
                for pos, value in pts:
                    bins.setdefault(int(pos // BIN_TICKS) * BIN_TICKS, []).append(value - mean)
                profiles[side][name] = {
                    "direction": "forward" if sign > 0 else "reverse",
                    "mean_proxy_pwm": round(mean, 1),
                    "bins": {b: round(statistics.fmean(v), 1) for b, v in sorted(bins.items())},
                }
        offset = [offset[i] + raw["encoder_delta"][i] for i in (0, 1)]
    agreement: dict = {}
    for side, runs in profiles.items():
        names = list(runs)
        for a in range(len(names)):
            for b in range(a + 1, len(names)):
                x, y = runs[names[a]]["bins"], runs[names[b]]["bins"]
                common = sorted(set(x) & set(y))
                if len(common) >= 6:
                    r = pearson([x[c] for c in common], [y[c] for c in common])
                    agreement[f"{side}:{names[a]}~{names[b]}"] = {
                        "bins": len(common), "r": round(r or 0, 2)}
    return {"assumptions": {"load_gain": LOAD_GAIN_TICKS_PER_PWM, "bin_ticks": BIN_TICKS,
                            "chain": "final encoder_delta incl. coast, V3 order"},
            "profiles": profiles, "pairwise_shape_correlation": agreement}


def step_rows(raw: dict) -> list[dict]:
    """20 ms ID_V1 I lines: count-delta and edge-timed speed per control step."""
    out = []
    previous = None
    for entry in raw.get("steps", []):
        ms, lt, rt, le, re_, lp, rp = entry["values"]
        row = {"ms": ms, "ticks": (lt, rt), "edge_us": (le, re_), "pwm": (lp, rp),
               "count_speed": None, "edge_speed": None}
        if previous is not None:
            dt = counter_delta(ms, previous["ms"])
            if dt > 0:
                row["count_speed"] = tuple(
                    counter_delta(row["ticks"][i], previous["ticks"][i]) * 1000.0 / dt for i in (0, 1))
                edge = []
                for i in (0, 1):
                    dticks = counter_delta(row["ticks"][i], previous["ticks"][i])
                    dus = (row["edge_us"][i] - previous["edge_us"][i]) % 2**32
                    edge.append(dticks * 1e6 / dus if dticks and dus else None)
                row["edge_speed"] = tuple(edge)
        out.append(row)
        previous = row
    return out


def fit_step(times_ms: list[float], speeds: list[float], before: float) -> dict | None:
    """Grid-fit v = before + gain_step * (1 - exp(-(t - delay)/tau)), t from duty change."""
    if len(times_ms) < 10:
        return None
    best = None
    after = statistics.fmean(speeds[len(speeds) // 2:])
    for delay in range(0, 61, 5):
        for tau in range(5, 505, 5):
            sse = 0.0
            for t, v in zip(times_ms, speeds):
                model = before if t < delay else before + (after - before) * (1 - math.exp(-(t - delay) / tau))
                sse += (v - model) ** 2
            if best is None or sse < best[0]:
                best = (sse, delay, tau)
    rms = math.sqrt(best[0] / len(times_ms))
    return {"delay_ms": best[1], "tau_ms": best[2], "after": round(after, 1), "fit_rms": round(rms, 1)}


def analyze_id(raw: dict, name: str = "") -> dict:
    """Offline summary of one ID_V1 duty-step record (no pass/fail criteria)."""
    sign = 1 if raw["targets"][0] > 0 else -1
    steps = step_rows(raw)
    result = {"record": name, "episode": raw.get("episode"), "reason": raw.get("reason"),
              "timer1_fast": raw.get("timer1_fast"), "schedule": raw.get("schedule"),
              "step_ms": None, "wheels": {}}
    gaps = [counter_delta(b["ms"], a["ms"]) for a, b in zip(steps, steps[1:])]
    result["control_dt_ms"] = (min(gaps), max(gaps)) if gaps else None
    schedule = raw.get("schedule") or []
    if len(schedule) < 2 or len(steps) < 20:
        result["incomplete"] = True
        return result
    levels = [tuple(abs(x) for x in entry[1:]) for entry in schedule[:2]]
    for i, side in enumerate(("left", "right")):
        a_level, b_level = levels[0][i], levels[1][i]
        a_rows = [r for r in steps if abs(r["pwm"][i]) == a_level and r["count_speed"]]
        change = next((k for k, r in enumerate(steps) if abs(r["pwm"][i]) == b_level), None)
        if not a_rows or change is None:
            result["wheels"][side] = {"incomplete": True}
            continue
        t_change = steps[change]["ms"]
        # Level A settled: last 0.6 s before the change.
        a_window = [r for r in a_rows if 0 < counter_delta(t_change, r["ms"]) <= 600]
        b_rows = [r for r in steps[change:] if r["count_speed"] and abs(r["pwm"][i]) == b_level]
        b_window = [r for r in b_rows if counter_delta(r["ms"], t_change) >= 600]
        if len(a_window) < 10 or len(b_window) < 10:
            result["wheels"][side] = {"incomplete": True}
            continue
        va = statistics.fmean(sign * r["count_speed"][i] for r in a_window)
        vb = statistics.fmean(sign * r["count_speed"][i] for r in b_window)
        edge_a = [sign * r["edge_speed"][i] for r in a_window if r["edge_speed"][i] is not None]
        fit = fit_step([counter_delta(r["ms"], t_change) for r in b_rows],
                       [sign * r["count_speed"][i] for r in b_rows], va)
        result["step_ms"] = t_change
        result["wheels"][side] = {
            "duty": (a_level, b_level),
            "speed_a": round(va, 1), "speed_b": round(vb, 1),
            "incremental_gain_ticks_s_per_pwm": round((vb - va) / (b_level - a_level), 2),
            "speed_a_count_sd": round(statistics.pstdev(sign * r["count_speed"][i] for r in a_window), 1),
            "speed_a_edge_sd": round(statistics.pstdev(edge_a), 1) if len(edge_a) > 2 else None,
            "step_fit": fit,
        }
    return result


def main(argv: list[str]) -> int:
    if argv[1:2] == ["--id"]:
        out = [analyze_id(load_any(path), Path(path).name) for path in argv[2:]]
        json.dump(out, sys.stdout, indent=1)
        sys.stdout.write("\n")
        return 0
    if argv[1:] == ["--position"]:
        json.dump(position_profiles(), sys.stdout, indent=1)
        sys.stdout.write("\n")
        return 0
    paths = argv[1:] or sorted(glob.glob("docs/measurements/R19_*.json"))
    results = []
    for path in paths:
        raw = load(path)
        if raw is None or not raw.get("targets") or raw["targets"][0] == 0:
            continue
        results.append(analyze(raw, Path(path).name))
    json.dump(results, sys.stdout, indent=1)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
