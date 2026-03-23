# Project Guideline: Priority-Aware Multi-Drone Coordination

## Overview

A fleet of hospital drones transporting time-sensitive medical cargo must navigate shared airspace to reach a **single landing pad**. The cargo includes organs, blood products, medications, and equipment. Because only one drone can land at a time, the pad creates a **structured bottleneck**.

This project uses the **SMGLib framework** to evaluate different multi-robot coordination strategies. It replaces standard first-come-first-served landing policies with a **dynamic priority system**. Drones evaluate their cargo criticality, time sensitivity, distance, and patient acuity to calculate a real-valued **priority score**.

To resolve airspace conflicts before they become critical, the system integrates a **pre-contact negotiation module**. When drones detect a conflict, they exchange status details. The project compares standard rule-based arbitration against an **LLM-enhanced decision model** to dictate which drone yields.

**Ultimate Goal:** Optimize global patient welfare by minimizing total weighted delivery delays.

---

## Implementation Phases and Demos

### Phase 1 — Baseline Bottleneck Simulation

**Objective:** Establish the core simulation environment and enforce the single-pad constraint.

**Implementation:**
- Adapt the n-SMG shared doorway concept to represent a single-coordinate landing pad.
- All drones share one goal location (the pad), with a mutual exclusion constraint — only one drone may occupy it at a time.

**Demo 1 — Greedy Baseline:**
Two drones approach the pad. The drone closest to the landing zone lands immediately while the other waits.

| What to show | Expected behavior |
|---|---|
| 2 drones, 1 pad | Closer drone lands first, farther drone idles |
| Metric | Total delivery time, pad idle time |

---

### Phase 2 — Dynamic Priority Integration

**Objective:** Introduce the medical logistics scoring logic.

**Implementation:**
- Build a `priority_calculator()` function using weighted inputs:

| Factor | Description | Example Weight |
|---|---|---|
| **Cargo type** | Organ > blood product > medication > equipment | Categorical weight |
| **Time until expiration** | Lower remaining time = higher urgency | Inverse relationship |
| **Flight distance** | Closer drones have natural advantage, but priority can override | Distance penalty |
| **Patient acuity** | Critical > urgent > routine | Severity score |

- Priority score is a **real-valued number** computed per drone, updated dynamically as conditions change.

**Demo 2 — Priority Override:**
Three converging drones. A farther drone carrying a critical organ overtakes a closer drone carrying routine equipment.

| What to show | Expected behavior |
|---|---|
| 3 drones, mixed cargo | Organ drone lands first despite being farther |
| Metric | Priority inversion avoided, weighted delivery delay |

---

### Phase 3 — Advanced Navigation and Holding Patterns

**Objective:** Implement complex spatial behaviors for yielding agents.

**Implementation:**
- **VIP drones (high priority):** Direct flight path to landing pad — shortest route, no deviation.
- **Yielding drones (low priority):** Circular holding pattern — drone aborts approach and enters orbit until pad is clear.
- Holding pattern parameters: orbit radius, orbit speed, re-approach trigger condition.

**Demo 3 — Yield and Circle:**
A low-priority drone on final approach aborts its landing and enters a holding pattern to clear the airspace for a newly spawned critical drone.

| What to show | Expected behavior |
|---|---|
| 2 drones, late-spawning VIP | Low-priority drone breaks off, circles, VIP lands directly |
| Metric | VIP delay = 0, yield drone total path increase |

---

### Phase 4 — Pre-Contact Negotiation Module

**Objective:** Integrate arbitration methods to resolve landing conflicts.

**Implementation:**
- **Proximity trigger:** When two drones are within a configurable distance threshold of the pad (or each other), they exchange status data:
  - Current position, velocity, heading
  - Priority score
  - Estimated time to pad
- **Arbitration toggle:** Switch between two modes:

| Mode | How it decides who yields |
|---|---|
| **Rule-based** | Deterministic comparison of priority scores — highest score proceeds |
| **LLM-enhanced** | Drone status data is formatted as a prompt; LLM returns a binary yield decision with reasoning |

**Demo 4 — Side-by-Side Comparison:**
Run the exact same conflict scenario twice — once with rule-based arbitration, once with LLM arbitration. Compare yielding decisions and outcomes.

| What to show | Expected behavior |
|---|---|
| Same scenario, 2 modes | Decisions may differ in edge cases (similar priority scores) |
| Metric | Decision latency, outcome quality, consistency |

---

### Phase 5 — Lookahead Scheduling and Global Metrics

**Objective:** Shift from greedy immediate decisions to optimized global throughput.

**Implementation:**
- **Lookahead algorithm:** Plans the landing queue over a configurable future time horizon.
  - Considers all drones' ETAs, priority scores, and holding pattern costs.
  - Produces an optimal landing sequence that minimizes total weighted delay.
- **Data logging:** Track and report:

| Metric | Description |
|---|---|
| **Pad utilization** | Fraction of time the pad is actively in use vs. idle |
| **Priority inversion rate** | How often a lower-priority drone lands before a higher-priority one |
| **Total weighted delivery delay** | Sum of (priority_weight × delivery_delay) across all drones |
| **Patient welfare proxy** | Mathematical score based on delivery times and cargo criticality |

**Demo 5 — Global Welfare Test:**
A VIP drone voluntarily enters a brief holding pattern. This delay allows multiple medium-priority drones to land in quick succession, demonstrating improved overall throughput compared to a strict greedy approach.

| What to show | Expected behavior |
|---|---|
| 4+ drones, mixed priority | VIP delays slightly, 3 medium drones land fast, net welfare improves |
| Metric | Global weighted delay lower than greedy baseline |

---

## Team Assignments

### Member 1 — Jinyoon Kim: Environment and Simulation Lead

| Responsibility | Details |
|---|---|
| Framework adaptation | Adapt SMGLib to create the hospital landing pad bottleneck environment |
| Movement controllers | Program flight controllers (direct paths, holding patterns) |
| Flight paths | Code VIP direct routes and circular holding orbits |
| Platform | C++ and Linux environments where applicable |

### Member 2 — Logic and Arbitration Lead

| Responsibility | Details |
|---|---|
| Priority calculator | Build `priority_calculator()` with cargo, time, distance, acuity inputs |
| Negotiation triggers | Develop pre-contact proximity detection and status exchange |
| LLM integration | Integrate LLM (Python) to process drone status data and return binary yield decisions |

### Member 3 — Evaluation and Scenario Lead

| Responsibility | Details |
|---|---|
| Test case design | Define starting coordinates, spawn times, cargo assignments for each demo |
| Data tracking | Implement logging for priority inversions, pad utilization, delivery delays |
| Welfare scoring | Formulate mathematical proxy for patient outcomes based on delivery times |

---

## Phase-to-File Mapping (SMGLib)

| Phase | Primary files to modify |
|---|---|
| Phase 1 | `uav.py` (landing constraint), `app2_standardized.py` (pad scenario), `utils.py` (environment) |
| Phase 2 | New `priority.py` module, `uav.py` (add cargo/priority attributes) |
| Phase 3 | `uav.py` (holding pattern dynamics), `dynamic.py` (orbit controller) |
| Phase 4 | New `negotiation.py` module, `test.py` (arbitration integration) |
| Phase 5 | New `scheduler.py` module, `utils.py` (new metrics) |

---

## PR Strategy

| PR | Branch | Phase | Content |
|---|---|---|---|
| #1 | `feature/landing-pad-env` | Phase 1 | Single-pad bottleneck environment, 2-drone greedy baseline |
| #2 | `feature/priority-system` | Phase 2 | Priority calculator, cargo attributes on agents |
| #3 | `feature/holding-patterns` | Phase 3 | Direct VIP paths, circular holding orbits, yield behavior |
| #4 | `feature/negotiation-module` | Phase 4 | Proximity trigger, rule-based + LLM arbitration toggle |
| #5 | `feature/lookahead-scheduler` | Phase 5 | Lookahead queue optimizer, global metrics, welfare scoring |

---

## Key Design Decisions

1. **Landing pad = shared doorway bottleneck** — reuses SMGLib's n-SMG doorway concept with a single-point goal.
2. **Priority is dynamic, not static** — scores update as time-to-expiration decreases and distances change.
3. **Holding pattern = circular orbit** — yielding drones don't just stop; they fly a predictable pattern, which is more realistic and visually demonstrable.
4. **LLM is a toggle, not a replacement** — rule-based is the default; LLM is an optional comparator for ambiguous edge cases.
5. **Global welfare > individual TTG** — the final metric is total weighted delivery delay across all drones, not any single drone's performance.
