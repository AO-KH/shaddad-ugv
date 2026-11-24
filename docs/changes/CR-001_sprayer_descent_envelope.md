<!--
doc_no: NASL-SHD-CR-001
doc_type: CHANGE REQUEST
title: Shaddad UGV: CR-001 the descent envelope of the full-tank sprayer
subtitle: Class 1 change: the 20° design slope with the sprayer and a full tank
summary: With a full tank, the sprayer cannot be stopped safely at a usable speed on the 20° design ramp. On the straight, the robot measures 21-21.6° (the body settles nose-down), where the safe start speed of a brakes-locked stop is 0.05-0.10 m/s, so the speed cap gives a target of 0. The owner, as the change board, decides how the envelope changes.
version: 1.2
date: 2026-11-16
status: Approved
history: 1.0 | 2026-11-15 | Raised from card G2-DSC (the system runs and the structure re-tabulation)
history: 1.1 | 2026-11-15 | Decision of the change board: option 1, an operating limit for the sprayer
history: 1.2 | 2026-11-16 | Editorial: the team named by its roles; effort in engineering passes
prepared_by: the lead systems engineer
reviewer: [TBD]
approver: the project owner (the change board)
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV CR-001
file: NASL_CR-001_Shaddad_Sprayer_Descent_Envelope_v1.2
outdir: work/reports/changes
request: Card G2-DSC (the downhill speed cap); the owner's decisions of 2026-11-15 (always-on cap; sprayer on the deck; keep the latch)
related: NASL-EMS-HBK-001 section 8.1 (change control); the step reports of the structure and integration engineers on card G2-DSC
-->
# CR-001: the descent envelope of the full-tank sprayer

**Class 1:** it changes the operating envelope of a configuration (the design slope with the sprayer), and so it changes a requirement and its scenarios (NASL-EMS-HBK-001 Table 19). **Status:** approved on 2026-11-15 (option 1); in implementation on card G2-DSC.

## 1. The change

- **What:**
  - how the full-tank sprayer may descend the 20° design ramp;
  - what scenario S2 tests.
- **Why:** card G2-DSC built the always-on speed cap that the owner chose: a brakes-locked stop may start only at or below the safe speed for the slope and the fitted implement. The system runs and the structure re-tabulation then showed:
  - **The robot measures more than the terrain.** On the 20° ramp the body settles nose-down. The robot measures 21-21.6° on the straight, and 0.5° of IMU error is allowed on top.
  - **Little speed is safe there.** With a full tank (316 kg, centre of gravity 0.375 m high), a stop that skids the locked tracks may start only at 0.05-0.10 m/s at those pitches without tipping over the front. The limiting case is coasting with no motor torque, the sharp-edge track end and farm soil.
  - **The robot then stops.** The MCU sets its speed target 0.10 m/s below the cap, so the target is 0. The robot stops at the top of the straight; S2 cannot be completed with the sprayer.
  - **The robot cannot tell a full tank from an empty one,** so the full-tank row applies whenever the sprayer is fitted.
- **Raised by:** the lead, 2026-11-15, from card G2-DSC.

## 2. Impact analysis

Table 1. The impact, from the structure, MCU, GNC and integration engineers' reports.

| Area | Impact |
|---|---|
| Requirements | the operating envelope of the sprayer on slopes; the 20° design slope stays for no implement, the mower and the tiller forward |
| Scenarios | S2, S2b, S2c and S2d all run the full-tank sprayer on the 20° ramp: they change with any option |
| Design | option 2 changes the MCU and the GNC (the braked-state tables); option 3 changes the structure model and needs a test on the robot |
| Tests | t_gnc_mission's fixed check of the 10/15/20° entries; the descent cases of t_mcu_control, which follow the table |
| Safety | every option keeps every brakes-locked stop at or below a safe speed; none relaxes a safety margin without evidence |
| Effort | option 1: one engineering pass on the scenarios and the documents; option 2: several passes of MCU and GNC work and a new class of tests; option 3: one pass now, plus the slope brake test on the robot (card T-04) |

A second defect is fixed inside the card (class 2) whatever the option: at a hill start the robot rolls back about 0.09 m/s when the brakes release, and the MCU's speed prediction trips it. The MCU engineer can drop the prediction term for about 0.5 s after the brakes release while the speed reference is held at zero.

## 3. Options

1. **An operating limit for the sprayer (recommended).**
   - The sprayer descends slopes up to about 18.5°, where the cap allows a usable speed.
   - The 20° design slope stays for no implement, the mower and the tiller forward.
   - S2 runs the 20° ramp without an implement (or with the mower), and a sprayer variant runs a ramp the sprayer may use.
   - The limit goes into the requirements and the operating concept.
   - *Consequence:* the safe and simple choice; the sprayer is not used on the steepest slopes.
2. **Braked-state tables.**
   - The MCU and the GNC use the higher braked-state values while the drive can brake: about 0.35 m/s forward at 21° and 0.25 at 22°.
   - *Consequence:* the sprayer keeps the 20° slope, but slowly. A drive fault at the cap on 20° still allows only about 0.30 m/s. This means more MCU and GNC work and a new class of tests.
3. **A rounded track end, backed by a slope brake test.**
   - Replace the sharp-edge bound with the rounded track end of the real track, which raises the tip speed by 12-20 %.
   - *Consequence:* still slow on 20°. It rests on a brake test on the robot (card T-04), and until then the cap stays as it is.

A later card can recover the envelope for every option: a controlled stop without skidding, which needs the drive to brake, as the structure engineer proposed.

## 4. Decision of the change board

**Approved: option 1, an operating limit for the sprayer.** The project owner decided on 2026-11-15, in the owner's words: "Limit sprayer to ~18.5° (Recommended)".
- **The full-tank sprayer** descends slopes up to about 18.5°.
- **The 20° design slope** stays for no implement, the mower and the tiller forward.
- **S2** runs the 20° ramp without the sprayer, plus a sprayer case on a slope it may use.
- **The limit** goes into the requirements and the operating concept.
- **The owner also approved** the effort to finish card G2-DSC ("Yes, finish").

## 5. Implementation and closure

- **Implementation:** card G2-DSC (the scenarios, the MCU hill-start fix, the GNC test, the reruns, the docs, the review and NASL-SHD-ER-005).
- **Verification:** the reruns of S2 to S2d, with the park criterion of the structure report (section 6).
- **Baseline:** the decision row D74, and the requirement change in the register.
