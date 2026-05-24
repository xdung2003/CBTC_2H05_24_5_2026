# ATC Simulation Parameter Baseline

## 1. Purpose

This document records the parameter baseline used by the simulator, while keeping a strict distinction between:

- values grounded in the supplied signalling/project documents
- values derived from those documents
- simulator assumptions chosen for modelling convenience

This file is intentionally conservative about what may be called `Documented`.

Status definitions:

- `Documented`: explicitly supported by the supplied source documents
- `Derived`: inferred from documented architecture or operating rules, but not directly stated as a numeric source value
- `Assumed`: simulator engineering choice pending authoritative project data

## 2. Source-Aligned Baseline Constraints

The following constraints are directly supported by the supplied references and shall govern the parameter baseline.

| Item | Baseline | Status | Source/Notes |
|---|---:|---|---|
| Mainline protected automatic mode | `ATO under ATP supervision` | Documented | PIC and UJV both describe ATO as subordinate to ATP |
| Mainline protected manual mode | `LMD under ATP supervision` | Documented | PIC defines LMD as manual mode supervised by ATP |
| Restricted/degraded mode ceiling | `25 km/h` for `CMD/CMD25` | Documented | PIC and UJV explicitly state CMD/CMD25 speed ceiling |
| Depot operating principle | `Restricted/manual, not normal CBTC ATO` | Documented | UJV states depot is not CBTC-equipped like mainline |
| Dynamic speed restriction authority | `TSR managed via Line Controller / ATS interface` | Documented | UJV assigns TSR management to LC with ATS interface |
| Static infrastructure data domain | `slope, speed limits, stations, beacons, points, signals` | Documented | UJV static guideway database definition |
| Dynamic authority data domain | `MA / EOA, TSR, variants` | Documented | UJV states MA and TSR are sent dynamically to train |
| Signalling headway target context | `135 s for 4-car ATO operation` | Documented | PIC states selected operating mode must satisfy 135-second headway target |

## 3. Simulation Classification Rule

Unless a numeric value is explicitly present in the provided source set, it shall not be marked `Documented`.

In particular, the current source set does **not** provide authoritative numeric confirmation for:

- brake build-up times
- service and emergency deceleration guarantees
- adhesion coefficients
- odometry confidence envelopes
- balise spacing values
- ATP reaction times
- overlap distances in metres for the simulator
- ZC communication periods and timeout values
- station stopping offsets in metres

These values may still exist in the simulator, but they are `Assumed` until a cited project source is added.

## 4. Train and Rolling Stock Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| Train formation reference for operating target | `4 cars` | Documented | Explicitly referenced by the PIC headway requirement |
| `AW0` mass | `220000 kg` | Assumed | Present in current simulator baseline, but not confirmed by the supplied source documents reviewed here |
| `AW2` mass | `291600 kg` | Assumed | Same as above |
| `AW3` mass | `331960 kg` | Assumed | Same as above |
| Train length | `60 m` default | Assumed | Current simulator default only; scenario override may apply |
| Nominal service brake acceleration | `1.12 m/s^2` | Assumed | Current model value, not a certified project brake guarantee in supplied docs |
| Nominal emergency brake acceleration | `1.25 m/s^2` | Assumed | Current model value, not a certified project brake guarantee in supplied docs |

## 5. Mode and Operational Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| Primary automatic operating mode | `ATO` | Documented | Normal mainline mode |
| Protected manual operating mode | `LMD` | Documented | Equivalent protection intent to ATO per PIC narrative |
| Restricted manual mode ceiling | `25 km/h` | Documented | CMD/CMD25 ceiling |
| ATO departure prerequisite | `DOO / departure authorisation required` | Documented | Station departure in ATO requires operational confirmation |
| Degraded fall-back when ATO/LMD unavailable | `CMD/CMD25` | Documented | Present in supplied project documents |
| ATO operational headway target | `135 s` | Documented | Project operating target, not an automatic proof that simulator achieves it |

## 6. Infrastructure Data Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| Permanent speed restrictions (`PSR`) | Scenario/static guideway data | Derived | UJV states PSR belongs to static on-board guideway data |
| Temporary speed restrictions (`TSR`) | Runtime/trackside update | Documented | UJV assigns TSR to LC/ATS dynamic management |
| Gradient / slope data | Scenario/static guideway data | Documented | UJV lists slope as part of the static guideway database |
| Station locations | Scenario/static guideway data | Documented | UJV lists stations in the static guideway database |
| Beacon locations | Scenario/static guideway data | Documented | UJV lists beacons in the static guideway database |
| Signal / point objects | Scenario/static guideway data | Documented | UJV lists these as static ATC objects |

## 7. Movement Authority and Separation Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| Separation principle for CBTC interpretation | `Moving block using absolute braking-distance principle` | Documented | Supported by theory material and CBTC references |
| Movement authority producer | `Trackside ATC / ZC using route and train-state information` | Documented | UJV ZC role |
| Onboard movement supervision | `ATP` | Documented | MA/EOA is supervised on the train by ATP logic |
| Safety margin between trains | `60.0 m` | Assumed | Current simulator reserve, not a cited project number |
| Moving-block overlap reserve | `50.0 m` | Assumed | Current simulator reserve, not a cited project number |
| Stop SvL offset beyond EOA | `1.0 m` | Assumed | Model-specific monitored-stop convention |
| Stop target offset behind SvL | `0.75 m` | Assumed | Model-specific stop target convention |

The source material confirms the existence of overlap and protected stopping concepts, but does not justify the current simulator metre values in this file.

## 8. Braking and ATP Modelling Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| ATP safety role | `Vital / fail-safe` | Derived | Aligned with source architecture and signalling safety intent |
| ATO safety role | `Non-vital under ATP` | Derived | Supported structurally by source documents |
| Adhesion factor | `0.82` | Assumed | Current simulator value only |
| Brake build-up time | `1.5 s` | Assumed | No authoritative numeric support in reviewed source set |
| Minimum deceleration floor | `0.15 m/s^2` | Assumed | Numerical model floor only |
| ATP `P` reaction margin | `2.5 s` | Assumed | Simulator internal parameter |
| ATP `W` reaction margin | `0.8 s` | Assumed | Simulator internal parameter |
| ATP `SBI` reaction margin | `1.2 s` | Assumed | Simulator internal parameter |
| ATP `EBI` reaction margin | `1.6 s` | Assumed | Simulator internal parameter |
| ATP indication delay | `0.8 s` | Assumed | Simulator internal parameter |
| High-speed time-margin gain | `0.35` | Assumed | Simulator conservatism tuning only |

The current source basis supports the need for protected braking supervision, but not the exact internal curve constants used by the simulator.

## 9. Positioning and Localisation Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| Onboard localisation support | `Beacons / balises + odometry` | Documented | UJV lists Eurobalises and train localisation functions |
| Position uncertainty envelope | `4.0 m` | Assumed | Current model value only |
| Balise spacing | `200 m` | Assumed | No reviewed source value confirms this spacing |
| Maximum odometry error between balises | `5.5 m` | Assumed | Model value only |
| Odometer drift growth | `5% of distance travelled`, capped | Assumed | Model value only |
| Standstill speed epsilon | `0.05 m/s` | Assumed | Numerical detection threshold |
| Standstill drift limit | `2.0 m` | Assumed | Model protection threshold |
| Rollback protection distance | `1.0 m` | Assumed | Model protection threshold |

## 10. ATO Control Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| ATO obeys ATP limit | Yes | Documented | Core source requirement |
| ATO control speed resolution | `0.5 km/h` | Assumed | Simulator implementation choice |
| Speed estimation resolution | `1.0 km/h` | Assumed | Simulator implementation choice |
| Vital speed margin | `0.5 km/h` | Assumed | Simulator implementation choice |
| Speed low-pass filter time constant | `0.35 s` | Assumed | Simulator implementation choice |
| ATO control strategy | Gain-scheduled PID with jerk-limited command | Assumed | Implementation choice, not source requirement |
| Docking speed | `4.0 km/h` | Assumed | Simulator implementation choice |
| Final approach max speed | `5.0 km/h` | Assumed | Simulator implementation choice |
| Final approach min speed | `1.0 km/h` | Assumed | Simulator implementation choice |
| Jog max distance | `15.0 m` | Assumed | Simulator implementation choice |
| Jog speed | `1.0 km/h` | Assumed | Simulator implementation choice |
| Jerk limit | `0.75 m/s^3` | Assumed | Reasonable comfort target, but not evidenced numerically by the reviewed source set |

## 11. Release, Creep and Fine-Stop Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| Release zone | `100.0 m` | Assumed | Simulator-specific behaviour |
| Release speed | `18.0 km/h` | Assumed | Simulator-specific behaviour |
| Creep / release cap | `25.0 km/h` | Assumed | The 25 km/h source value applies to CMD/CMD25, not automatically to simulator release logic |
| Release handover start | `16.0 m` | Assumed | Simulator-specific behaviour |
| Final stop brake zone | `8.0 m` | Assumed | Simulator-specific behaviour |
| Release entry margin | `1.5 km/h` | Assumed | Simulator-specific behaviour |

These values should not be presented as project railway requirements unless a cited operating rule is added.

## 12. Communication and Timing Baseline

| Item | Baseline | Status | Notes |
|---|---:|---|---|
| Main simulation cycle | `0.1 s` | Assumed | Code constant, not a project-source timing requirement |
| Wayside-to-train authority update | Every simulation step | Assumed | Current simulator simplification |
| DCS one-way packet delay | `0.05 .. 0.35 s` random | Assumed | Current simulator simplification |
| Train state reporting period | Every simulation step | Assumed | Current simulator simplification |
| Timeout behaviour | `1.0 s` | Assumed | Current simulator fail-safe threshold |
| ATP MA extrapolation | `1.5 s` | Assumed | Current simulator diagnostic/protection parameter |
| Position-report latency | `0.6 s` | Assumed | Current simulator communications parameter |

The source material supports continuous communication and fail-safe restriction on subsystem/version faults, but does not provide these exact timing values in the reviewed set.

## 13. Open Project Data Needed

The following data should be sourced from authoritative project material before any of the corresponding simulation values can be promoted beyond `Assumed`:

- certified rolling stock mass tables and train length
- guaranteed service and emergency braking performance
- adhesion assumptions by weather and contamination state
- brake build-up and reaction-time decomposition
- balise placement strategy and odometry confidence budgets
- exact MA, TSR and communication cycle timing
- overlap distances and route-protection distances for the target line
- stopping tolerance and platform-door interface timing
- wet/dry operational parameter adjustments

## 14. Change Control

- Any parameter change that affects simulator behaviour shall update this file.
- If a parameter changes from `Assumed` to `Documented`, the specific source document shall be cited in the same row.
- If a parameter is required for implementation but has no source support yet, it shall remain `Assumed` and be traceable to a simulator design decision rather than to railway-system theory.
