from __future__ import annotations

import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import SUBSYSTEMS.physics as physics
from CONFIG.config import TRAIN_MASS_KG


def test_traction_and_grade_acceleration():
    speed = physics.kmh_to_ms(30.0)
    flat = physics.acceleration_from_forces_ms2(
        speed,
        TRAIN_MASS_KG,
        traction_command=1.0,
        gradient=0.0,
    )
    uphill = physics.acceleration_from_forces_ms2(
        speed,
        TRAIN_MASS_KG,
        traction_command=1.0,
        gradient=0.03,
    )
    downhill = physics.acceleration_from_forces_ms2(
        speed,
        TRAIN_MASS_KG,
        traction_command=1.0,
        gradient=-0.03,
    )
    if flat <= 0.0:
        raise AssertionError("full traction on flat track should accelerate the train")
    if not uphill < flat < downhill:
        raise AssertionError("grade force should reduce uphill acceleration and increase downhill acceleration")


def test_davis_resistance_and_speed_dependent_traction():
    low_speed = physics.kmh_to_ms(10.0)
    high_speed = physics.kmh_to_ms(80.0)
    if physics.davis_resistance_force_n(high_speed, TRAIN_MASS_KG) <= physics.davis_resistance_force_n(low_speed, TRAIN_MASS_KG):
        raise AssertionError("Davis resistance should increase with speed")
    if physics.traction_force_n(high_speed, TRAIN_MASS_KG) >= physics.traction_force_n(low_speed, TRAIN_MASS_KG):
        raise AssertionError("available traction force should reduce at higher speed")
    if physics.running_resistance_accel_ms2(high_speed) != physics.davis_resistance_accel_ms2(high_speed):
        raise AssertionError("legacy running_resistance_accel_ms2 should remain Davis-compatible")


def test_braking_distance_from_force():
    speed = physics.kmh_to_ms(60.0)
    service_distance = physics.braking_distance_from_force_m(speed, TRAIN_MASS_KG, "service")
    emergency_distance = physics.braking_distance_from_force_m(speed, TRAIN_MASS_KG, "emergency")
    downhill_emergency = physics.braking_distance_from_force_m(speed, TRAIN_MASS_KG, "emergency", gradient=-0.04)
    if not service_distance > emergency_distance > 0.0:
        raise AssertionError("service braking distance should be longer than emergency braking distance")
    if not downhill_emergency > emergency_distance:
        raise AssertionError("downhill gradient should increase emergency braking distance")


def test_energy_accounting():
    speed = physics.kmh_to_ms(40.0)
    traction = physics.longitudinal_force_balance(
        speed,
        TRAIN_MASS_KG,
        traction_command=0.75,
        brake_command=0.0,
        distance_m=100.0,
    )
    braking = physics.longitudinal_force_balance(
        speed,
        TRAIN_MASS_KG,
        traction_command=0.0,
        brake_command=0.5,
        brake_mode="service",
        distance_m=100.0,
    )
    if traction.traction_work_j <= 0.0 or traction.brake_work_j != 0.0:
        raise AssertionError("traction work should be positive only when traction is commanded")
    if braking.brake_work_j <= 0.0 or braking.traction_work_j != 0.0:
        raise AssertionError("brake work should be positive only when braking is commanded")
    if braking.acceleration_ms2 >= 0.0:
        raise AssertionError("service braking force balance should decelerate the train")


def test_runtime_adapter_uses_force_balance_and_keeps_legacy_fallback():
    speed = physics.kmh_to_ms(25.0)
    traction = physics.resolve_runtime_dynamics(
        speed,
        TRAIN_MASS_KG,
        commanded_accel_ms2=0.5,
        force_balance_enabled=True,
    )
    if traction.used_fallback:
        raise AssertionError("runtime adapter unexpectedly used legacy fallback")
    if traction.traction_command <= 0.0 or traction.forces.traction_force_n <= 0.0:
        raise AssertionError("positive acceleration command should map to traction force")
    if traction.forces.brake_force_n != 0.0:
        raise AssertionError("traction command should not apply brake force")

    braking = physics.resolve_runtime_dynamics(
        speed,
        TRAIN_MASS_KG,
        commanded_accel_ms2=-0.7,
        brake_mode_hint="service",
        force_balance_enabled=True,
    )
    if braking.brake_command <= 0.0 or braking.forces.brake_force_n <= 0.0:
        raise AssertionError("negative acceleration command should map to brake force")
    if braking.acceleration_ms2 >= 0.0:
        raise AssertionError("braking command should decelerate through force balance")

    legacy = physics.resolve_runtime_dynamics(
        speed,
        TRAIN_MASS_KG,
        commanded_accel_ms2=0.5,
        gradient=0.01,
        force_balance_enabled=False,
    )
    expected = physics.legacy_runtime_acceleration_ms2(speed, 0.5, 0.01)
    if not legacy.used_fallback:
        raise AssertionError("disabled force balance should use legacy fallback")
    if abs(legacy.acceleration_ms2 - expected) > 1e-9:
        raise AssertionError("legacy fallback acceleration changed")


def main() -> int:
    test_traction_and_grade_acceleration()
    test_davis_resistance_and_speed_dependent_traction()
    test_braking_distance_from_force()
    test_energy_accounting()
    test_runtime_adapter_uses_force_balance_and_keeps_legacy_fallback()
    print("physics regression ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


