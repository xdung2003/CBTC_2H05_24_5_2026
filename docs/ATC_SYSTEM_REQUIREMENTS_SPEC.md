# ATC System Requirements Specification

## 1. Scope

This document defines the baseline signalling and ATC requirements used by the project documentation and simulator reviews. It is aligned to the source documents provided for the project domain, not only to the current simulator implementation.

The intent is to distinguish:

- source-aligned signalling theory and project requirements
- simulation-specific abstractions used for analysis or software implementation

## 2. Source Basis

The following documents are treated as the primary technical basis for this specification:

- `Chuong 1 Cac thanh phan va thuat ngu co ban.pdf`: signalling components, terminology, signal classes, station/route concepts, overlap and protection length
- `c2ktth.pdf`: train separation principles, absolute braking distance, fixed block, moving block, continuous supervision assumptions
- `PIC-TEC-TTS-SNO-L00-74111-V-3A`: project signalling technical and functional requirements for ATP, ATO, ATS, IXL and operating modes
- `UJV-BAS-TTS-SIG-L00-40206-E-4A-D300-2.00.pdf`: system architecture, interlocking behaviour, moving-block ATC roles, ATS/IXL/ATC interfaces, depot constraints

Where the simulator uses a stronger simplification than the source documents, that simplification shall be stated explicitly and shall not be treated as a railway-system requirement.

## 3. System Boundary

### 3.1 Signalling System Composition

For this project domain, the signalling system shall be understood as a coordinated set of:

- ATC
- Interlocking / CBI / IXL
- ATS
- DCS / communication network
- train detection and localisation devices
- points, signals, beacons and associated wayside equipment

Per the provided project documents, ATC is not a stand-alone speed governor. It is part of a larger signalling system and exchanges information with ATS and IXL.

### 3.2 ATC Composition

ATC shall be treated as comprising, at minimum:

- ATP: vital train protection and supervision
- ATO: automatic train operation under ATP supervision
- trackside ATC functions such as ZC and LC
- carborne controller functions on the train

This follows the UJV architecture definition where ATC is composed of `ATP, ATO, LC and ZC`, with carborne and trackside partitions.

### 3.3 ATS Role

ATS shall be treated as the supervisory and traffic-regulation layer, normally associated with OCC and local ATS workstations. ATS may:

- supervise train movements
- request route setting or cancellation through interlocking interfaces
- issue operational regulation orders
- manage TSR-related status through the ATC interface
- support degraded-operation control by operators

ATS is not the vital protection authority. Vital route and movement protection remain the responsibility of IXL and ATP/ATC.

## 4. Operating Domain and Modes

### 4.1 Mainline and Depot Separation

The project shall distinguish clearly between:

- mainline CBTC operation
- depot / shunting / transfer-area operation

The supplied documents state that the depot is not CBTC-equipped in the same way as the mainline and that depot operation relies on dedicated interlocking and restricted/manual modes. Any simulator narrative that treats depot automatic operation as equivalent to mainline CBTC is not source-aligned.

### 4.2 Mainline Driving Modes

The baseline operational modes are:

- `ATO`: automatic train operation under ATP supervision
- `LMD`: manual driving with ATP supervision
- `CMD` / `CMD25`: restricted manual mode with low speed ceiling used as degraded mode and in depot-related operation

The source documents consistently require at least these three operating modes. ATO and LMD are normal protected modes on the mainline. CMD/CMD25 is a constrained or degraded manual mode.

### 4.3 ATO Behaviour

In this project context, ATO shall:

- operate only under ATP supervision
- obey speed limits and protective constraints provided by ATP/ATC
- support automatic running between stations on the mainline
- support station stop functions, through-running, hold orders and scheduled departure logic
- support operational adaptation such as wet/dry rail cases where required by the project specification

Departure from station in ATO shall require operational authorization such as DOO/departure confirmation as described by the supplied PIC document.

### 4.4 LMD Behaviour

LMD shall be treated as protected manual driving:

- traction and braking are commanded by the driver
- protection and speed supervision remain under ATP
- the train remains governed by protected movement logic rather than pure line-of-sight driving

LMD is therefore not equivalent to unrestricted manual mode.

### 4.5 CMD / CMD25 Behaviour

CMD / CMD25 shall be treated as restricted manual operation with a low speed ceiling. The supplied UJV document defines `CMD25` explicitly as manual driving with ATP speed ceiling at `25 km/h`.

The documentation shall not claim that CMD provides the same traffic-performance or signalling capability as ATO/LMD. It is a degraded or restricted mode, including depot use.

## 5. Interlocking Principles

### 5.1 Route Safety

IXL/CBI shall be treated as the subsystem that generates and protects safe routes. In particular it shall:

- process route requests from ATS or automatic route logic
- verify route compatibility
- detect conflicts
- set and lock points
- lock routes and sub-routes
- control signals according to route state
- exchange route and occupancy status with ATC and ATS

This is a core source requirement and shall not be omitted from system-level documentation.

### 5.2 Conflict Protection

Interlocking shall prevent at least:

- front collision routes
- flank collision routes
- unsafe switch movements under locked or occupied conditions
- unsafe route creation when overlap or sub-route conditions are not satisfied

### 5.3 Sectional Release

When a train proceeds over a locked route, the route behind the train shall be releasable section by section, subject to detection and locking conditions. This principle is explicitly described in the UJV interlocking requirements.

### 5.4 Approach Locking and Route Cancellation

The system documentation shall include the approach-locking principle:

- route cancellation first restores the origin signal to restrictive state
- if no train is within the approach locking section, route release may occur immediately
- if a train is in the approach locking section, release depends on timer logic or proof that the train will stop safely before the restrictive signal

Any simplified simulator behaviour shall be documented as a simplification.

### 5.5 Overlap

The overlap concept shall be retained as a fundamental signalling protection concept:

- overlap is an area beyond a signal used to protect train approach and stopping safety
- overlaps may constrain conflicting route setting
- overlap locking and release are part of interlocking safety behaviour

The project documents do not support removing overlap from the theory baseline, even if the simulator uses a simplified overlap constant.

## 6. Train Separation and Movement Authority

### 6.1 Separation Principles

The documentation shall recognise the distinction between:

- relative braking distance
- absolute braking distance
- fixed block distance
- moving block

The supplied theory document states that CBTC and ETCS Level 3 are associated with continuous-information systems using the absolute braking distance principle and moving-block behaviour.

### 6.2 Moving Block Preconditions

Moving-block operation shall not be described as position-only spacing. It depends on continuous or frequent trusted information including, as applicable:

- train position
- train integrity / completeness assumptions
- braking capability assumptions
- communication continuity
- route / point / protection-area status from wayside systems

This matters because the source documents explicitly tie moving-block benefits to continuous supervision and information exchange.

### 6.3 Fixed Block Relevance

The documentation shall not imply that moving block fully eliminates fixed-block reasoning in every operational context. The theory material shows that fixed block remains relevant for some railway applications, degraded operation and train-detection assurance.

### 6.4 Movement Authority

For a CBTC interpretation in this project, the movement authority / EOA concept shall be treated as:

- generated by trackside ATC using route, occupancy, point and protection information
- transmitted to the carborne controller
- supervised on board by ATP

The ZC manages moving-block status and movement authorities using train reports and information from interlocking and detection subsystems.

## 7. ATP Requirements

At system level, ATP shall:

- protect train movement against unsafe speed and unsafe authority overrun
- supervise train movement with respect to permitted movement limits
- remain the final protection layer over ATO
- support protected operation in ATO and LMD
- support restricted/degraded operation policies where applicable

The source documents support ATP as the vital protection function. They do not, by themselves, mandate the exact internal curve taxonomy used by the current simulator.

## 8. Static and Dynamic Infrastructure Data

The ATC model may use static guideway data including:

- track geometry
- gradients
- permanent speed restrictions
- stations
- points and signals
- beacon locations

Dynamic operational data may include:

- movement authority
- temporary speed restrictions
- route status
- train reports

This matches the UJV description of static guideway data on board and dynamic TSR/authority data sent from trackside.

## 9. Signals and Station Principles

The supporting theory documents require the documentation to preserve conventional signalling concepts even in CBTC context:

- different signal roles such as main, entry, exit, route and shunting signals
- restrictive default states in stations unless a safe route is established
- distinction between main tracks and subsidiary tracks
- protection lengths and overlap concepts associated with stop signals and protected movement

Accordingly, any ATS or simulator display that presents stations, sidings or depot exits should preserve the notion that route safety is tied to aligned path, locked points and protected stopping distance.

## 10. Degraded Operation

The documentation shall describe degraded operation explicitly. At minimum:

- loss of ATO does not remove ATP protection in protected modes
- CMD/CMD25 exists as a degraded or restricted manual mode
- depot and some non-mainline movements rely on restricted/manual operating principles rather than normal CBTC ATO
- communication or subsystem failures require fail-safe restriction rather than permissive continuation

## 11. Simulation-Specific Abstractions

The following items may exist in the simulator, but they shall be documented as modelling choices unless separately justified by a cited requirement:

- exact ATP curve families such as `I/P/W/SBI/SBD/EBI/EBD`
- internal release-curve state machines
- software-specific headway formulas
- custom observability fields and logs
- simplified departure gates for source/station visual elements
- lane graphics and ATS display interaction mechanisms

These can be useful engineering artefacts, but they are not by themselves project requirements from the provided railway documents.

## 12. Documentation Rule

Any future requirement statement in project docs shall be classified as one of:

- `Source Requirement`: directly supported by the supplied railway/signalling references
- `Project Assumption`: needed for the simulator or project scope but not explicit in the source references
- `Simulation Abstraction`: implementation detail or verification construct used by the software model

This rule is required to keep the documentation technically honest and traceable.
