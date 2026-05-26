from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import GUI.main_gui as main_gui
from MONTECARLO.monte_carlo import MonteCarloConfig, apply_sample_to_scenario, run_batch, sample_inputs
from CONFIG.scenario_loader import normalize_scenario
import random


def make_scenario():
    return normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 1000, "gradient": 0.0, "psr_kmh": 70}]},
            "scheduled_stops": [{"name": "S500", "pos_m": 500.0, "length_m": 160.0, "capacity": 2, "dwell_s": 20.0}],
            "source_trains": [{"name": "SRC", "capacity": 2, "total_trains": 2}],
            "headway": {"mode": "fixed", "target_headway_s": 90.0},
            "trains": [],
        }
    )


def test_monte_carlo_batch_returns_statistical_results():
    summary = run_batch(
        make_scenario(),
        MonteCarloConfig(runs=3, max_sim_time_s=80.0, seed=7),
        main_gui.Simulation,
        main_gui,
    )
    if len(summary.results) != 3:
        raise AssertionError("Monte Carlo runner did not complete requested runs")
    if not summary.table_lines() or "Monte Carlo Statistical Analysis" not in summary.table_lines()[0]:
        raise AssertionError("Monte Carlo summary table missing title")
    samples = [result.sample for result in summary.results]
    if not all("dcs_packet_delay_s" in sample and "adhesion_factor" in sample for sample in samples):
        raise AssertionError("Monte Carlo samples missing randomized DCS/adhesion parameters")
    if not any(result.trains_per_hour >= 0.0 for result in summary.results):
        raise AssertionError("Monte Carlo result missing capacity KPI")


def test_monte_carlo_dwell_sampling_respects_minimum():
    config = MonteCarloConfig(seed=11)
    rng = random.Random(config.seed)
    scenario = make_scenario()
    sample = sample_inputs(config, rng, scenario)
    sampled = apply_sample_to_scenario(scenario, sample, rng, config)
    dwell_values = [float(stop["dwell_s"]) for stop in sampled["scheduled_stops"]]
    if not dwell_values or min(dwell_values) < config.min_passenger_dwell_s:
        raise AssertionError("Monte Carlo sampled dwell below minimum passenger dwell")


def test_route_release_delay_does_not_clamp_source_headway():
    scenario = make_scenario()
    scenario["source_trains"] = [{"name": "SRC", "capacity": 4, "total_trains": 4}]
    scenario["headway"] = {"mode": "fixed", "target_headway_s": 120.0}
    previous = main_gui.TURNOUT_LOCK_S
    try:
        main_gui.TURNOUT_LOCK_S = 180.0
        sim = main_gui.Simulation(scenario)
        for _ in range(int(700.0 / main_gui.DT)):
            sim.step()
            if len(sim.headway_manager.stats.dispatch_times_s) >= 4:
                break
    finally:
        main_gui.TURNOUT_LOCK_S = previous
    actual = sim.headway_manager.stats.actual_headways_s
    if len(actual) < 3:
        raise AssertionError("source headway test did not dispatch all source trains")
    avg_actual = sum(actual) / len(actual)
    if avg_actual > 130.0:
        raise AssertionError(f"route release lock clamped source headway to {avg_actual:.1f}s")


def main() -> int:
    test_monte_carlo_batch_returns_statistical_results()
    test_monte_carlo_dwell_sampling_respects_minimum()
    test_route_release_delay_does_not_clamp_source_headway()
    print("monte carlo regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


