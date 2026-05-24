# ATC Verification Matrix

## 1. Purpose

This matrix verifies the source-aligned requirements defined in [ATC_SYSTEM_REQUIREMENTS_SPEC.md](/C:/Users/xuan1/Desktop/how%20to%20learn%20fullstacks/ATP/docs/ATC_SYSTEM_REQUIREMENTS_SPEC.md).

It separates:

- verification of signalling principles derived from the supplied references
- verification of simulator-specific modelling assumptions

Verification method legend:

- `A`: analysis / document review / traceability review
- `T`: targeted simulation test
- `R`: runtime observation / logging review

## 2. Source-Aligned System Verification

| Req ID | Verification Intent | Method | Pass Criteria |
|---|---|---|---|
| `SYS-ARCH-001` | Confirm docs describe signalling as ATC + ATS + IXL + DCS + field equipment, not ATP alone | A | System docs preserve the multi-subsystem architecture stated by the source documents |
| `SYS-ARCH-002` | Confirm ATC is described as ATP + ATO + trackside/carborne ATC roles | A | Docs do not reduce ATC to only ATO or only ATP |
| `SYS-ARCH-003` | Confirm ATS is described as supervisory rather than vital train protection | A | Docs assign traffic supervision/regulation to ATS and vital protection to ATP/IXL |
| `OPS-MODE-001` | Confirm normal mainline modes are ATO and LMD, with CMD/CMD25 as restricted/degraded mode | A | Docs preserve the three-mode operating baseline |
| `OPS-MODE-002` | Confirm depot is treated separately from mainline CBTC automatic operation | A | Docs state depot relies on dedicated interlocking and restricted/manual operation |
| `OPS-MODE-003` | Confirm CMD25 restricted speed concept is preserved | A,T | Documentation cites 25 km/h ceiling; simulator mode cap does not exceed that value |
| `ATO-OPS-001` | Confirm ATO is subordinate to ATP | A,T | Docs state ATP supervision over ATO; in simulation, ATO cannot sustain unsafe command over ATP protection |
| `ATO-OPS-002` | Confirm ATO departure requires station authorization / DOO concept | A,T | Docs preserve DOO/departure authorization; simulation station departure flow requires explicit release condition |
| `ATO-OPS-003` | Confirm ATO station functions include stop, through, hold and operational regulation concepts | A,T | Docs include these functions and test scenarios exercise them where implemented |
| `LMD-OPS-001` | Confirm LMD is protected manual driving rather than unrestricted sight mode | A | Docs state ATP supervision remains active in LMD |
| `IXL-ROUTE-001` | Confirm interlocking route-setting responsibility is documented | A | Docs state route requests are processed by IXL/CBI with point and route locking |
| `IXL-ROUTE-002` | Confirm route conflict protection is documented | A | Docs include protection against front/flank conflict and unsafe switch usage |
| `IXL-ROUTE-003` | Confirm sectional route release is documented | A,T | Docs require release behind the train section by section; simulation route occupancy model, if present, is consistent |
| `IXL-ROUTE-004` | Confirm approach locking and route cancellation principles are documented | A | Docs include restrictive restoration, timer/proof-of-stop logic, and approach-locking section concept |
| `IXL-ROUTE-005` | Confirm overlap is retained as a signalling protection concept | A | Docs describe overlap as downstream protective area affecting route safety |
| `SEP-PRIN-001` | Confirm docs distinguish relative braking, absolute braking, fixed block and moving block | A | All four principles appear correctly and are not conflated |
| `SEP-PRIN-002` | Confirm moving block is tied to continuous trusted information, not only nominal position reports | A | Docs include communication, train state and wayside status dependencies |
| `SEP-PRIN-003` | Confirm fixed-block reasoning is still acknowledged for degraded or non-equivalent contexts | A | Docs do not claim that moving block eliminates all fixed-block concepts |
| `MA-ARCH-001` | Confirm movement authority / EOA is described as trackside-generated and onboard-supervised | A | Docs state ZC/ATC generates authority and ATP supervises it on train |
| `DATA-ARCH-001` | Confirm static guideway data and dynamic TSR/authority split is documented | A | Docs separate onboard static data from dynamic TSR/authority updates |
| `SIG-OPS-001` | Confirm station/signal documentation preserves route, stop-signal and shunting concepts | A | Docs include signal roles and protected route logic in stations/depot contexts |
| `DEG-OPS-001` | Confirm degraded operation is treated as fail-safe restriction, not permissive continuation | A,T | Docs state failures lead to restricted/degraded handling; simulation enters restricted or stop behaviour as designed |

## 3. Simulation-Specific Verification

These rows verify software-model assumptions that may be useful for the simulator, but are not direct railway-source requirements on their own.

| Req ID | Verification Intent | Method | Pass Criteria |
|---|---|---|---|
| `SIM-ATP-001` | Confirm ATP curve synthesis is independent of ATO piloting helper logic | A | ATP curve generation path does not depend on ATO target shaping |
| `SIM-ATP-002` | Confirm ATP can override ATO | T,R | Overspeed or authority violation causes ATP intervention despite ATO traction command |
| `SIM-ATP-003` | Check conservative brake model assumption influences ATP curves | T | Lower adhesion, higher delay or weaker braking yields a more restrictive curve set |
| `SIM-ATP-004` | Check downhill gradient produces more restrictive supervision than level track | T | Same target on downgrade yields lower protected speeds or longer braking demand |
| `SIM-ATP-005` | Confirm simulator exposes its internal curve families consistently | A,R | Runtime state consistently exposes the configured curve set such as `I/P/W/SBI/SBD/EBI/EBD` |
| `SIM-ATP-006` | Confirm release supervision, if modelled, remains inside the protected stop envelope | T,R | Release or creep logic cannot overrun protected stop conditions |
| `SIM-ATO-001` | Check normal ATO running stays below ATP envelope | T,R | Normal operation does not repeatedly trigger ATP intervention under nominal scenarios |
| `SIM-ATO-002` | Check stopping accuracy remains within configured tolerance | T,R | Final stop error remains inside the simulator tolerance |
| `SIM-MA-001` | Check moving-block headway changes when braking assumptions or latency are worsened | T | Weaker braking or higher latency increases required separation in the model |
| `SIM-COMMS-001` | Check communication timeout produces fail-safe degraded behaviour | T,R | Packet timeout drives restricted or emergency state according to simulator design |
| `SIM-POS-001` | Check odometry uncertainty grows between correction points and shrinks on correction | T,R | Error band grows between updates and reduces when correction event occurs |
| `SIM-HOLD-001` | Check rollback or protected-standstill drift trips the train if the model includes these hazards | T,R | Reverse drift beyond threshold triggers the configured protection response |

## 4. Regression Scenario Families

The following scenario families should be maintained where the simulator supports them:

- `VS-001`: level-track target speed reduction
- `VS-002`: downhill target speed reduction
- `VS-003`: commanded station stop from line speed
- `VS-004`: approach to restrictive limit / stop point
- `VS-005`: moving-block follower under dense headway
- `VS-006`: low-adhesion braking case
- `VS-007`: odometry correction after drift growth
- `VS-008`: communication-loss degraded mode
- `VS-009`: route / station hold and departure authorization
- `VS-010`: depot or restricted-mode operation

## 5. Maintenance Rule

No safety-significant design or code change should be accepted without checking:

- whether it changes a `Source-Aligned System Verification` row
- whether it only changes a `Simulation-Specific Verification` row
- whether a new assumption should be promoted or demoted between those two groups
