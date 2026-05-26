from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import GUI.main_gui as main_gui
from CONFIG.scenario_loader import normalize_scenario


ATP_EOA_BRAKE_BUFFER_M = getattr(main_gui, "ATP_EOA_BRAKE_BUFFER_M", 0.0)


def make_single_train():
    scenario = normalize_scenario(
        {
            "track": {"segments": [{"start_m": 0, "end_m": 3000, "gradient": 0.0, "psr_kmh": 80}]},
            "trains": [{"id": "ATP1", "start_pos": 0.0, "drive_mode": "ATO"}],
            "source_trains": [],
            "scheduled_stops": [],
        }
    )
    return main_gui.Simulation(scenario).trains[0]


def vital_brake_mass(train) -> float:
    return max(train.mass, main_gui.AW3_MASS_KG) / max(main_gui.ATP_ADHESION_FACTOR, 1e-6)


def braking_distance_with_buildup_m(speed_ms: float, mass_kg: float, brake_mode: str, gradient: float, build_s: float) -> float:
    force_n = main_gui.EMERGENCY_FORCE_N if brake_mode == "emergency" else main_gui.BRAKE_FORCE_N
    decel = main_gui.conservative_brake_decel_ms2(force_n, mass_kg, 1.0)
    decel = max(main_gui.ATP_MIN_DECEL_MS2, decel - max(0.0, -gradient) * main_gui.G)
    return main_gui.stopping_distance_with_buildup(speed_ms, decel, build_s)


def speed_for_braking_distance_ms(
    distance_m: float,
    mass_kg: float,
    brake_mode: str,
    gradient: float,
    build_s: float,
    reaction_s: float,
    extra_margin_m: float,
    max_speed_ms: float,
) -> float:
    force_n = main_gui.EMERGENCY_FORCE_N if brake_mode == "emergency" else main_gui.BRAKE_FORCE_N
    decel = main_gui.conservative_brake_decel_ms2(force_n, mass_kg, 1.0)
    decel = max(main_gui.ATP_MIN_DECEL_MS2, decel - max(0.0, -gradient) * main_gui.G)
    usable_m = max(0.0, distance_m - extra_margin_m)
    low = 0.0
    high = max(0.0, max_speed_ms)
    for _ in range(48):
        mid = 0.5 * (low + high)
        required = main_gui.stopping_distance_with_buildup(mid, decel, build_s) + mid * reaction_s
        if required <= usable_m:
            low = mid
        else:
            high = mid
    return low


def test_physics_curve_stops_before_eoa_at_multiple_speeds():
    train = make_single_train()
    mass = vital_brake_mass(train)
    for speed_kmh in (20.0, 40.0, 60.0, 80.0):
        speed = main_gui.kmh_to_ms(speed_kmh)
        required = braking_distance_with_buildup_m(
            speed,
            mass,
            "emergency",
            0.0,
            main_gui.ATP_BRAKE_BUILDUP_S,
        )
        eoa_distance = (
            required
            + speed * main_gui.ATP_EBI_REACTION_S
            + main_gui.POS_UNCERT_M
            + ATP_EOA_BRAKE_BUFFER_M
            + 5.0
        )
        allowed = speed_for_braking_distance_ms(
            eoa_distance,
            mass,
            "emergency",
            0.0,
            main_gui.ATP_BRAKE_BUILDUP_S,
            main_gui.ATP_EBI_REACTION_S,
            main_gui.POS_UNCERT_M + ATP_EOA_BRAKE_BUFFER_M,
            main_gui.kmh_to_ms(120.0),
        )
        if allowed + main_gui.kmh_to_ms(0.05) < speed:
            raise AssertionError(f"EBI curve should allow a physics stop before EOA at {speed_kmh:.0f} km/h")

        short_distance = max(0.0, eoa_distance - 25.0)
        short_allowed = speed_for_braking_distance_ms(
            short_distance,
            mass,
            "emergency",
            0.0,
            main_gui.ATP_BRAKE_BUILDUP_S,
            main_gui.ATP_EBI_REACTION_S,
            main_gui.POS_UNCERT_M + ATP_EOA_BRAKE_BUFFER_M,
            main_gui.kmh_to_ms(120.0),
        )
        if short_allowed >= allowed:
            raise AssertionError("EBI permitted speed should decrease as EOA distance shrinks")


def test_ebi_intervenes_before_eoa_when_speed_is_unsafe():
    for speed_kmh in (20.0, 40.0, 60.0, 80.0):
        train = make_single_train()
        speed = main_gui.kmh_to_ms(speed_kmh)
        mass = vital_brake_mass(train)
        required = braking_distance_with_buildup_m(
            speed,
            mass,
            "emergency",
            0.0,
            main_gui.ATP_BRAKE_BUILDUP_S,
        )
        unsafe_eoa = max(20.0, 0.55 * required)
        train.pos = 0.0
        train.reported_pos = 0.0
        train.prev_pos = 0.0
        train.speed = speed
        train.filtered_speed = speed
        train.estimated_speed = speed
        train.vital_speed = speed
        train.receive_safe_packet(
            main_gui.SafeMovementPacket(
                eoa_m=unsafe_eoa,
                tsr_kmh=80.0,
                variants={
                    "gradient": 0.0,
                    "next_speed_limit_kmh": 80.0,
                    "next_speed_limit_dist_m": float("inf"),
                },
                issued_time_s=0.0,
            ),
            0.0,
        )
        train.step(0.0)
        if train.pos >= unsafe_eoa:
            raise AssertionError("test setup should still be before EOA after one ATP step")
        if train.atp_action != "EBI" or not train.emg_latch:
            raise AssertionError(f"ATP should trigger EBI before EOA when {speed_kmh:.0f} km/h is unsafe")


def main() -> int:
    test_physics_curve_stops_before_eoa_at_multiple_speeds()
    test_ebi_intervenes_before_eoa_when_speed_is_unsafe()
    print("atp brake curve regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


