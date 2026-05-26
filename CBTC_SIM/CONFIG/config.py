HOST = "127.0.0.1"
PORT = 50007

DT = 0.1  # 100 ms

TRAIN_LENGTH_M = 60.0
AW0_MASS_KG = 220000.0     # empty train mass (AW0) ~220 t
AW2_MASS_KG = 291600.0     # enormal passengr load (AW2) ~291.6 t
AW3_MASS_KG = 331960.0     # crush load (AW3) ~331.96 t
TRAIN_MASS_KG = AW2_MASS_KG

MAX_AXLE_LOAD_T = 15.0
AXLE_COUNT = 12
MAX_AXLE_LOAD_KG = MAX_AXLE_LOAD_T * 1000.0

MAX_SPEED_KMH = 80.0
MAX_TUNNEL_SPEED_KMH = 80.0

A_MAX = 1.2        # m/s^2 traction capability at low speed
A_BRAKE = 1.12     # m/s^2 tuned service brake capability for 100->0 km/h stop performance
A_EMERGENCY = 1.25  # m/s^2 tuned emergency brake capability while keeping service/emergency separation
MAX_JERK_MS3 = 0.75  # maximum longitudinal jerk for passenger comfort
ROTATING_MASS_FACTOR = 1.06  # equivalent mass allowance for wheels, motors and gear inertia

# Speed-dependent train running resistance, expressed as acceleration terms:
# a_resist = A + B*v + C*v^2. Grade resistance is handled separately by track gradient.
RUNNING_RESIST_A_MS2 = 0.012
RUNNING_RESIST_B_S1 = 0.00035
RUNNING_RESIST_C_INV_M = 0.00002

# Forces (N). Derived from nominal accel and nominal operating mass.
TRACTION_FORCE_N = TRAIN_MASS_KG * A_MAX
BRAKE_FORCE_N = TRAIN_MASS_KG * A_BRAKE
EMERGENCY_FORCE_N = TRAIN_MASS_KG * A_EMERGENCY

SAFETY_MARGIN_M = 60.0
OVERLAP_M = 50.0

# Brake build-up time (s)
BRAKE_BUILDUP_S = 1.5

# Runtime dynamics mode. When enabled, Train.step resolves acceleration through
# physics.longitudinal_force_balance and records force telemetry. The legacy
# acceleration path remains available as a fallback.
USE_FORCE_BALANCE_RUNTIME = True

LOG_EVERY_S = 1.0

