# Shaddad UGV - R&D life cycle (tailored for a small team)

**What this is.** How the Shaddad project runs to a defined end through technical review gates, in the systems-engineering order of MIL-STD-1521B / IEEE 15288.2 (NASA uses near-identical gates). The user decided on 2026-11-15 to apply the tailored version for a small R&D team. The work is defined as task cards on the dashboard's Project tab (`tools/dashboard/project_cards.json`), each card under the gate it serves. **The project is done when every card is done and every gate is passed.**

## 1. The tailoring

As the life-cycle chart's tailoring box says:
- **One combined system review.** SRR and SFR are merged into gate G1.
- **PDR and CDR per subsystem** (gates G2 and G3), not at system level.
- **The verification matrix starts at G1 and is never dropped.** It is what makes FCA possible, and it shows early when a requirement cannot be verified.
- **PRR only if production starts.** It is not applicable now.
- **The parallel software track** (SSR after SFR, then SW PDR and SW CDR) runs inside the MCU, Companion and GNC subsystem gates.

## 2. The gates

| Gate | Question | Bring | Locks |
|---|---|---|---|
| **G1 SRR + SFR** (system) | Do we know what to build? Can every requirement be verified? Is the functional architecture sound and allocated to the subsystems? | the operating concept; the system spec; a verification method per requirement (Test, Analysis, Inspection or Demonstration); the functional breakdown; requirement allocation and traceability; the ICD; the subsystem specs; risks and TBDs | the **functional baseline**: the agreed system requirements |
| **G2 PDR** (per subsystem) | Does the chosen design meet the requirements with acceptable risk? | trade studies; the HW/SW architecture; mass, power, timing and thermal budgets; prototype (model) results; long-lead items | the **allocated baseline** ("design-to") |
| **G3 CDR** (per subsystem) | Is the detailed design complete enough to build, code and integrate? | the builders and models (about 90 %); the detailed SW design; analyses closed; qualification test plans; the build approach | the **initial product baseline** ("build-to") |
| **Build** | Fabricate, code, integrate | built to the CDR baseline, then integrated and tested | - |
| **G4 TRR** (per test campaign) | Are we ready to start formal verification testing? | approved procedures; the test-article configuration; calibrated equipment; the safety plan; pass/fail criteria | the **go-ahead** for each campaign: simulation, bench/HIL, field |
| **G5 FCA / SVR** (system) | Does the tested system meet every requirement? | the closed verification matrix; test and analysis evidence; deviations and waivers | **requirements verified** |
| (PRR) | Can we build it repeatably, at rate? | manufacturing, tooling, supply chain | not applicable now |
| **G7 PCA** (system) | Does what we built match the documentation? | the as-built robot against the model and docs; the software version description | the **final product baseline** |

**How each gate works:**
1. **Entry criteria met:** the documents and evidence are ready. The agents prepare the review package.
2. **Board review:** the lead and the user, with the independent reviewer agent. Action items are raised as RIDs (review item discrepancies).
3. **Decision:** pass, pass with actions, or a delta review.
4. **The gate review is recorded** as a PDF engineering report (the user's rule of 2026-11-15) and as a row in the decisions table.

**Configuration control:** after each baseline, changes go through formal configuration control. That means a decision row (D-number), snapshots, the test baseline and the docs update, as the agent workflow already does.

## 3. Where Shaddad stands (assessment of 2026-11-15)

The simulation model has reached about CDR maturity for most subsystems: all 17 build-plan tasks are built, with decisions D1 to D73 and a test suite. **The real-robot hardware is at PDR at best:**
- the controller board, the BMS, the ESC and its CAN protocol are open or assumed;
- the motor default, the pack cooling and the precharge are undecided.

The system baseline (G1) itself is out of date, because the specs describe the robot from before 2026-10-27 (R-32). **So the first gate to hold is G1, re-run on the as-built robot.**

| Gate | Status | Evidence | Gaps (review items) |
|---|---|---|---|
| G1 SRR + SFR | **re-baselined 2026-11-18** (card G1-01, D110: NASL-SHD-REQ-008 v2.0); the review G1-05 next | `docs/architecture.md` (scope, operating concept, D1-D73), 8 domain specs, `docs/build_plan.md`, the ICD, `docs/requirements/power.yaml` | specs describe the old robot (R-32, R-16, R-75, R-76); a verification matrix exists for power only; open TBDs: R-13 motor, R-14 ESC, R-15 BMS, R-59 board, R-04 cooling, R-09 precharge, decisions 2 and 5 |
| G2 PDR per subsystem | open, partly met | models, decisions, budgets in the specs | energy budget (R-19), thermal (R-04), hardware choices (R-14, R-15, R-59), turning model (R-18), the deck footprint (R-06) |
| G3 CDR per subsystem | open for simulation, partly met | builders and generated models; the critical fixes (ER-004) and the descent cap | 27 High and 46 Mid review items; analyses not closed (S3 endurance R-20, descent tip-over margins) |
| Build | models built; firmware generated, not flashed | T01-T17 done; the real robot assembled (XMachines) | MCU code from before D65 (R-60); Companion build fails (HybridAStar) |
| G4 TRR | not ready | the test suite, scenarios S0-S8, R1; row-test and live-drive procedures | 66 known failures, stale expectations (R-29), no subsystem Fast tests (R-28), blind regression checks (R-27), scenario staleness (R-26) |
| G5 FCA / SVR | not started | - | no verification matrix (G1) |
| G7 PCA | not started; known gaps | - | model against the as-built robot: R-13, R-14, R-15, R-59, R-04, tiller data; docs against the model: R-32, R-75, R-87 |

## 4. The path to done

1. **G1:** re-baseline the requirements on the as-built robot, build the verification matrix for every domain, settle the open system decisions, get the supplier data, and hold the combined review.
2. **G2, then G3, per subsystem**, in the order the risk sets: power and drive (stopping and thermal), MCU and electronics (the real board), sensors and navigation, GNC, structure, comms, integration. Each gate closes or waives its subsystem's review items.
3. **Build:** flash the firmware on the chosen board, build the Companion, instrument the robot.
4. **G4 for each campaign:** the simulation campaign, the bench/HIL campaign and the field campaign (row tests, a brake test on a slope, live drives).
5. **G5:** close the verification matrix, with deviations and waivers.
6. **G7:** audit the as-built robot against the model and the docs, and lock the final product baseline.

The task cards on the Project tab break these steps into work, each with its purpose, owner, deliverables, acceptance criteria and linked review items.
