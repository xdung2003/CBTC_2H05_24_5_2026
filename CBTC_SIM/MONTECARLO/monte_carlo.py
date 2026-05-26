from __future__ import annotations

import math
import random
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass, field
from statistics import mean
from typing import Any, Callable, Dict, Iterable, List


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _percentile(values: Iterable[float], percentile: float) -> float | None:
    data = sorted(float(value) for value in values if value is not None and math.isfinite(float(value)))
    if not data:
        return None
    if len(data) == 1:
        return data[0]
    rank = (len(data) - 1) * percentile
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return data[lower]
    ratio = rank - lower
    return data[lower] + (data[upper] - data[lower]) * ratio


def _recorded_station_dwell_s(record: Dict[str, Any], minimum_s: float) -> float | None:
    for key in ("passenger_dwell_s", "station_wait_s", "planned_dwell_s"):
        value = record.get(key)
        if value is None:
            continue
        try:
            dwell_s = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(dwell_s):
            return max(minimum_s, dwell_s)
    if record.get("train_id") is not None:
        return minimum_s
    return None


@dataclass
class MonteCarloConfig:
    runs: int = 100
    max_sim_time_s: float = 600.0
    seed: int | None = None
    simulation_time_scale: int = 50

    dwell_mean_s: float = 30.0
    extra_dwell_delay_std_s: float = 5.0
    min_passenger_dwell_s: float = 25.0
    dispatch_jitter_min_s: float = 1.0
    dispatch_jitter_max_s: float = 3.0
    target_headway_min_s: float = 120.0
    target_headway_max_s: float = 300.0
    route_release_delay_s: float = 180.0

    dcs_packet_delay_min_s: float = 0.1
    dcs_packet_delay_max_s: float = 0.5
    dcs_packet_loss_prob: float = 0.005
    dcs_mute_duration_min_s: float = 2.0
    dcs_mute_duration_max_s: float = 5.0

    odometry_drift_min_percent: float = 1.0
    odometry_drift_mode_percent: float = 2.0
    odometry_drift_max_percent: float = 3.0
    position_noise_mean_m: float = 2.0
    position_noise_std_m: float = 0.5
    docking_error_std_m: float = 0.5

    reaction_delay_mean_s: float = 2.0
    reaction_delay_std_s: float = 0.4
    brake_build_up_min_s: float = 1.0
    brake_build_up_max_s: float = 2.0
    brake_factor_margin: float = 1.2
    adhesion_low: float = 0.55
    adhesion_mode: float = 0.7
    adhesion_high: float = 1.0
    jerk_limit_min_ms3: float = 0.8
    jerk_limit_max_ms3: float = 1.0

    tsr_probability: float = 0.25
    tsr_speed_choices_kmh: tuple[float, ...] = (15.0, 25.0, 40.0)
    station_approach_release_max_kmh: float = 15.0


@dataclass
class MonteCarloRunResult:
    run_id: int
    sample: Dict[str, Any]
    avg_actual_headway_s: float | None
    p95_dispatch_headway_s: float | None
    avg_station_arrival_headway_s: float | None
    p95_station_arrival_headway_s: float | None
    avg_passenger_dwell_s: float | None
    trains_per_hour: float
    ebi_count: int
    sbi_count: int
    collision_count: int
    safe_margin_violations: int
    traction_work_kwh: float
    brake_work_kwh: float


@dataclass
class MonteCarloSummary:
    config: MonteCarloConfig
    results: List[MonteCarloRunResult] = field(default_factory=list)

    def values(self, field_name: str) -> List[float]:
        values: List[float] = []
        for result in self.results:
            value = getattr(result, field_name)
            if value is not None and math.isfinite(float(value)):
                values.append(float(value))
        return values

    def probability(self, predicate: Callable[[MonteCarloRunResult], bool]) -> float:
        if not self.results:
            return 0.0
        return sum(1 for result in self.results if predicate(result)) / len(self.results)

    def table_lines(self) -> List[str]:
        dispatch_headways = self.values("avg_actual_headway_s")
        station_headways = self.values("avg_station_arrival_headway_s")
        station_p95 = self.values("p95_station_arrival_headway_s")
        dwell_values = self.values("avg_passenger_dwell_s")
        tph_values = self.values("trains_per_hour")
        energy_values = self.values("traction_work_kwh")
        completed = len(self.results)
        lines = [
            "Monte Carlo Statistical Analysis",
            "-" * 96,
            f"Runs completed          : {completed} / {self.config.runs}",
            f"Statistical run speed   : x{max(1, int(self.config.simulation_time_scale))} headless batch",
            f"Target headway range    : {self.config.target_headway_min_s:.0f}s - {self.config.target_headway_max_s:.0f}s",
            f"DCS delay range         : {self.config.dcs_packet_delay_min_s:.2f}s - {self.config.dcs_packet_delay_max_s:.2f}s",
            f"Packet loss probability : {self.config.dcs_packet_loss_prob * 100.0:.3f}%",
            "",
            "Headway distribution",
            "-" * 96,
            f"Mean dispatch headway   : {_fmt(mean(dispatch_headways) if dispatch_headways else None)}",
            f"P50 dispatch headway    : {_fmt(_percentile(dispatch_headways, 0.50))}",
            f"P90 dispatch headway    : {_fmt(_percentile(dispatch_headways, 0.90))}",
            f"P95 dispatch headway    : {_fmt(_percentile(dispatch_headways, 0.95))}",
            f"Pr(avg dispatch <=120s) : {self.probability(lambda r: r.avg_actual_headway_s is not None and r.avg_actual_headway_s <= 120.0) * 100.0:.1f}%",
            f"Pr(avg dispatch <=300s) : {self.probability(lambda r: r.avg_actual_headway_s is not None and r.avg_actual_headway_s <= 300.0) * 100.0:.1f}%",
            "",
            "Station passenger service",
            "-" * 96,
            f"Mean station headway    : {_fmt(mean(station_headways) if station_headways else None)}",
            f"P95 station headway     : {_fmt(_percentile(station_p95, 0.95))}",
            f"Mean passenger dwell    : {_fmt(mean(dwell_values) if dwell_values else None)}",
            "",
            "Safety / reliability",
            "-" * 96,
            f"Pr(EBI > 0)             : {self.probability(lambda r: r.ebi_count > 0) * 100.0:.1f}%",
            f"Pr(SBI > 0)             : {self.probability(lambda r: r.sbi_count > 0) * 100.0:.1f}%",
            f"Pr(collision > 0)       : {self.probability(lambda r: r.collision_count > 0) * 100.0:.1f}%",
            f"Pr(safe margin breach)  : {self.probability(lambda r: r.safe_margin_violations > 0) * 100.0:.1f}%",
            "",
            "Capacity / energy",
            "-" * 96,
            f"Mean trains per hour    : {_fmt(mean(tph_values) if tph_values else None, unit=' tph')}",
            f"P95 traction work       : {_fmt(_percentile(energy_values, 0.95), unit=' kWh')}",
            "",
            "Recent runs",
            "-" * 96,
            "Run  avg_headway  station_p95  dwell_avg  tph   EBI SBI COL breach  sample",
        ]
        for result in self.results[-12:]:
            lines.append(
                f"{result.run_id:>3}  {_fmt(result.avg_actual_headway_s):>11}  "
                f"{_fmt(result.p95_station_arrival_headway_s):>11}  "
                f"{_fmt(result.avg_passenger_dwell_s):>9}  "
                f"{result.trains_per_hour:>4.1f}  {result.ebi_count:>3} {result.sbi_count:>3} "
                f"{result.collision_count:>3} {result.safe_margin_violations:>6}  "
                f"TH={result.sample.get('target_headway_s', 0.0):.0f}s DCS={result.sample.get('dcs_packet_delay_s', 0.0):.2f}s "
                f"adh={result.sample.get('adhesion_factor', 0.0):.2f}"
            )
        if self.results:
            lines.extend([
                "",
                "Collision notice per recent sample",
                "-" * 96,
            ])
            for result in self.results[-12:]:
                if result.collision_count > 0:
                    lines.append(f"Run {result.run_id}: COLLISION DETECTED {result.collision_count} time(s)")
                else:
                    lines.append(f"Run {result.run_id}: no collision detected")
        return lines


def _fmt(value: float | None, unit: str = "s") -> str:
    if value is None or not math.isfinite(float(value)):
        return "--"
    return f"{float(value):.1f}{unit}"


def sample_inputs(config: MonteCarloConfig, rng: random.Random, scenario: Dict[str, Any]) -> Dict[str, Any]:
    track_end = float(scenario.get("track_end_m", scenario.get("track_max_m", 1000.0)))
    track_start = float(scenario.get("track_min_m", 0.0))
    target_headway = rng.uniform(config.target_headway_min_s, config.target_headway_max_s)
    dcs_delay = rng.uniform(config.dcs_packet_delay_min_s, config.dcs_packet_delay_max_s)
    sample = {
        "target_headway_s": target_headway,
        "dispatch_jitter_s": rng.uniform(config.dispatch_jitter_min_s, config.dispatch_jitter_max_s),
        "dcs_packet_delay_s": dcs_delay,
        "dcs_packet_loss_prob": config.dcs_packet_loss_prob,
        "dcs_mute_duration_s": rng.uniform(config.dcs_mute_duration_min_s, config.dcs_mute_duration_max_s),
        "odometry_drift_percent": rng.triangular(
            config.odometry_drift_min_percent,
            config.odometry_drift_max_percent,
            config.odometry_drift_mode_percent,
        ),
        "position_noise_m": _clamp(rng.gauss(config.position_noise_mean_m, config.position_noise_std_m), 0.2, 6.0),
        "docking_error_m": abs(rng.gauss(0.0, config.docking_error_std_m)),
        "reaction_delay_s": _clamp(rng.gauss(config.reaction_delay_mean_s, config.reaction_delay_std_s), 0.5, 4.0),
        "brake_build_up_s": rng.uniform(config.brake_build_up_min_s, config.brake_build_up_max_s),
        "brake_factor": config.brake_factor_margin,
        "adhesion_factor": rng.triangular(config.adhesion_low, config.adhesion_high, config.adhesion_mode),
        "jerk_limit_factor": rng.uniform(config.jerk_limit_min_ms3, config.jerk_limit_max_ms3),
        "route_release_delay_s": config.route_release_delay_s,
        "station_approach_release_speed_kmh": rng.uniform(8.0, config.station_approach_release_max_kmh),
        "tsr_active": rng.random() < config.tsr_probability,
        "tsr_speed_kmh": rng.choice(config.tsr_speed_choices_kmh),
        "tsr_start_m": rng.uniform(max(0.0, track_start), max(1.0, track_end * 0.55)),
    }
    sample["tsr_end_m"] = min(track_end, sample["tsr_start_m"] + rng.uniform(200.0, 700.0))
    return sample


def apply_sample_to_scenario(
    scenario: Dict[str, Any],
    sample: Dict[str, Any],
    rng: random.Random,
    config: MonteCarloConfig | None = None,
) -> Dict[str, Any]:
    scenario = deepcopy(scenario)
    config = config or MonteCarloConfig()
    headway = dict(scenario.get("headway", {}) or {})
    if str(headway.get("mode", "fixed")).lower() == "off":
        headway["mode"] = "fixed"
    headway["target_headway_s"] = float(sample["target_headway_s"])
    scenario["headway"] = headway

    for idx, stop in enumerate(scenario.get("scheduled_stops", [])):
        base = float(stop.get("dwell_s", 30.0))
        if not math.isfinite(base) or base <= 0.0:
            base = 30.0
        stop["dwell_s"] = _clamp(
            base + rng.gauss(0.0, config.extra_dwell_delay_std_s),
            config.min_passenger_dwell_s,
            90.0,
        )

    jitter = float(sample["dispatch_jitter_s"])
    for train in scenario.get("trains", []):
        windows = list(train.get("dcs_mute_windows", []) or [])
        if rng.random() < max(0.0, min(1.0, float(sample["dcs_packet_loss_prob"]) * 20.0)):
            start = rng.uniform(30.0, max(31.0, float(sample.get("target_headway_s", 90.0)) * 3.0))
            windows.append({"start_s": start, "end_s": start + float(sample["dcs_mute_duration_s"])})
        train["dcs_mute_windows"] = windows
        if "start_pos" in train:
            train["start_pos"] = float(train["start_pos"]) - jitter * 0.1

    if sample.get("tsr_active"):
        scenario.setdefault("tsr_zones", [])
        scenario["tsr_zones"].append(
            {
                "start": float(sample["tsr_start_m"]),
                "end": float(sample["tsr_end_m"]),
                "speed": float(sample["tsr_speed_kmh"]),
            }
        )
    return scenario


@contextmanager
def patched_runtime(module: Any, sample: Dict[str, Any]):
    patch = {
        "DCS_DELAY_MIN_S": max(0.0, float(sample["dcs_packet_delay_s"]) - 0.05),
        "DCS_DELAY_MAX_S": float(sample["dcs_packet_delay_s"]) + 0.05,
        "DCS_TIMEOUT_S": float(sample["dcs_mute_duration_s"]),
        "DCS_PACKET_LOSS_PROB": float(sample["dcs_packet_loss_prob"]),
        "ODOMETER_ERROR_RATE": float(sample["odometry_drift_percent"]) / 100.0,
        "POS_UNCERT_M": float(sample["position_noise_m"]),
        "STATION_POS_UNCERT_M": min(float(sample["position_noise_m"]), 1.0),
        "PRECISE_STOP_POS_UNCERT_M": min(float(sample["docking_error_m"]), 0.8),
        "STOP_ACCURACY_TOL_M": max(0.3, float(sample["docking_error_m"])),
        "ATP_P_REACTION_S": float(sample["reaction_delay_s"]),
        "ATP_W_REACTION_S": max(0.5, float(sample["reaction_delay_s"]) * 0.4),
        "ATP_SBI_REACTION_S": max(0.8, float(sample["reaction_delay_s"]) * 0.6),
        "ATP_EBI_REACTION_S": max(1.0, float(sample["reaction_delay_s"]) * 0.8),
        "BRAKE_BUILDUP_S": float(sample["brake_build_up_s"]),
        "ATP_BRAKE_BUILDUP_S": max(1.0, float(sample["brake_build_up_s"]) - 0.2),
        "ATP_SERVICE_BRAKE_FACTOR": float(sample["brake_factor"]),
        "ATP_EMERGENCY_BRAKE_FACTOR": float(sample["brake_factor"]),
        "ATP_ADHESION_FACTOR": float(sample["adhesion_factor"]),
        "MAX_JERK_MS3": float(sample["jerk_limit_factor"]),
        "TURNOUT_LOCK_S": float(sample["route_release_delay_s"]),
        "RELEASE_SPEED_KMH": min(15.0, float(sample["station_approach_release_speed_kmh"])),
    }
    previous = {key: getattr(module, key) for key in patch if hasattr(module, key)}
    try:
        for key, value in patch.items():
            if hasattr(module, key):
                setattr(module, key, value)
        yield
    finally:
        for key, value in previous.items():
            setattr(module, key, value)


def _disable_event_logging(sim: Any) -> None:
    if hasattr(sim, "event_logging_enabled"):
        sim.event_logging_enabled = False
    for train in getattr(sim, "trains", []):
        train.event_logging_enabled = False
        if hasattr(train, "event_records"):
            train.event_records.clear()
        if hasattr(train, "pending_event_records"):
            train.pending_event_records.clear()


def _source_generation_complete(sim: Any) -> bool:
    return all(
        int(source.get("generated", 0)) >= int(source.get("total_trains", 0))
        for source in getattr(sim, "source_trains", [])
    )


def _train_done_for_monte_carlo(sim: Any, train: Any) -> bool:
    if float(getattr(train, "pos", 0.0)) >= float(getattr(sim, "track_end_m", 0.0)):
        return True
    if float(getattr(train, "speed", 0.0)) > 0.05:
        return False
    if getattr(train, "dwell_remaining_s", 0.0) != float("inf"):
        return False
    stop = getattr(train, "active_scheduled_stop", None)
    station_idx = sim._station_index_for_stop(stop) if stop is not None and hasattr(sim, "_station_index_for_stop") else None
    if hasattr(sim, "_is_final_scheduled_station") and sim._is_final_scheduled_station(station_idx):
        return True
    if hasattr(sim, "_is_terminal_station") and sim._is_terminal_station(station_idx):
        return True
    return False


def _monte_carlo_can_stop(sim: Any) -> bool:
    trains = list(getattr(sim, "trains", []))
    if not trains:
        return _source_generation_complete(sim)
    if all(float(getattr(train, "pos", 0.0)) >= float(getattr(sim, "track_end_m", 0.0)) for train in trains):
        return True
    return _source_generation_complete(sim) and all(_train_done_for_monte_carlo(sim, train) for train in trains)


def run_one(
    run_id: int,
    base_scenario: Dict[str, Any],
    config: MonteCarloConfig,
    simulation_cls: Any,
    runtime_module: Any,
    rng: random.Random,
    status: Callable[[Dict[str, Any]], None] | None = None,
) -> MonteCarloRunResult:
    if status is not None:
        status({"run_id": run_id, "runs": config.runs, "phase": "Randomizing Monte Carlo input variables"})
    sample = sample_inputs(config, rng, base_scenario)
    if status is not None:
        status({"run_id": run_id, "runs": config.runs, "phase": "Cloning scenario and applying randomized values"})
    scenario = apply_sample_to_scenario(base_scenario, sample, rng, config)
    safe_margin_violations = 0
    if status is not None:
        status({"run_id": run_id, "runs": config.runs, "phase": "Patching temporary CBTC runtime constants"})
    with patched_runtime(runtime_module, sample):
        if status is not None:
            status({"run_id": run_id, "runs": config.runs, "phase": "Building headless Simulation instance"})
        sim = simulation_cls(scenario)
        sim.analytics_detail_enabled = False
        sim.debug_trace_enabled = False
        _disable_event_logging(sim)
        for zone in scenario.get("tsr_zones", []):
            if zone not in sim.tsr_zones:
                sim.tsr_zones.append(dict(zone))
        steps = max(1, int(config.max_sim_time_s / runtime_module.DT))
        batch_size = max(1, int(config.simulation_time_scale))
        completed_steps = 0
        report_interval = max(batch_size, steps // 20)
        while completed_steps < steps:
            for _ in range(min(batch_size, steps - completed_steps)):
                sim.step()
                completed_steps += 1
                for train in sim.trains:
                    gap = getattr(train, "distance_to_train_ahead_m", None)
                    if gap is not None and gap < runtime_module.SAFETY_MARGIN_M:
                        safe_margin_violations += 1
                if _monte_carlo_can_stop(sim):
                    break
            if status is not None and (completed_steps >= steps or completed_steps % report_interval == 0):
                status(
                    {
                        "run_id": run_id,
                        "runs": config.runs,
                        "phase": "Running headless simulation",
                        "sim_completed_steps": completed_steps,
                        "sim_total_steps": steps,
                        "sim_time_s": sim.sim_time_s,
                    }
                )
            if _monte_carlo_can_stop(sim):
                break
        if status is not None:
            status({"run_id": run_id, "runs": config.runs, "phase": "Collecting KPI and station passenger metrics"})
        sim._update_analytics(include_station_metrics=True)
        analytics = sim.analytics
    station_headways = [
        value
        for station in analytics.get("station_passenger_metrics", [])
        for value in station.get("arrival_headways_s", [])
    ]
    station_dwell = [
        dwell_s
        for station in analytics.get("station_passenger_metrics", [])
        for record in station.get("arrivals", [])
        for dwell_s in [_recorded_station_dwell_s(record, config.min_passenger_dwell_s)]
        if dwell_s is not None
    ]
    dispatch_headways = analytics.get("actual_headways_s", [])
    return MonteCarloRunResult(
        run_id=run_id,
        sample=sample,
        avg_actual_headway_s=analytics.get("avg_actual_headway_s"),
        p95_dispatch_headway_s=_percentile(dispatch_headways, 0.95),
        avg_station_arrival_headway_s=mean(station_headways) if station_headways else None,
        p95_station_arrival_headway_s=_percentile(station_headways, 0.95),
        avg_passenger_dwell_s=mean(float(value) for value in station_dwell) if station_dwell else None,
        trains_per_hour=float(analytics.get("trains_per_hour", 0.0)),
        ebi_count=int(analytics.get("ebi_count", 0)),
        sbi_count=int(analytics.get("sbi_count", 0)),
        collision_count=int(analytics.get("collision_count", 0)),
        safe_margin_violations=safe_margin_violations,
        traction_work_kwh=float(analytics.get("traction_work_kwh", 0.0)),
        brake_work_kwh=float(analytics.get("brake_work_kwh", 0.0)),
    )


def run_batch(
    base_scenario: Dict[str, Any],
    config: MonteCarloConfig,
    simulation_cls: Any,
    runtime_module: Any,
    progress: Callable[[MonteCarloSummary], None] | None = None,
    status: Callable[[Dict[str, Any]], None] | None = None,
) -> MonteCarloSummary:
    rng = random.Random(config.seed)
    summary = MonteCarloSummary(config=config)
    for run_id in range(1, config.runs + 1):
        result = run_one(run_id, base_scenario, config, simulation_cls, runtime_module, rng, status=status)
        summary.results.append(result)
        if progress is not None:
            progress(summary)
    return summary

