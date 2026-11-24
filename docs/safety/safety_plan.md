<!--
doc_no: NASL-SHD-SAF-001
doc_type: SAFETY PLAN
title: Shaddad UGV: safety plan
subtitle: The safety life cycle of the mowing phase: the standards, the rating method, the independence of the assessment, the roles and the deliverables at each gate
summary: How the Shaddad project shows that the robot is acceptably safe for mowing within its envelope. The plan tailors the safety life cycle to the project's gates, names the jurisdiction and the dated standards with the binding Saudi layer, chooses the ISO 13849-1 risk graph to set the required performance levels, and sets who assesses, who checks and who approves. The first records under it are the hazard log NASL-SHD-HAZ-001 and its safety goals.
version: 1.4
date: 2026-11-18
status: Issued for review
history: 1.0 | 2026-11-16 | First issue, for the independent review (card G1-07)
history: 1.1 | 2026-11-16 | The independent review RID-G1-07-01 to -08: the exposure F and the avoidance P restated and applied as written (section 4), the reset requirements of the emergency stop (Table 1), the Safety Analysis Manager copy of the hazard log (section 8), the decisions for the owner (section 10)
history: 1.2 | 2026-11-16 | Approved by the project owner (D80). The owner's decisions D79 and D80 recorded in the summary, section 4 and section 10; no other change
history: 1.3 | 2026-11-18 | Card G1-01 part 2 (D103): section 3 and Table 1 give the jurisdiction and the dated standards with the binding Saudi layer; reasons 1 and 5 of section 4 and section 10 updated; no change to the method, the life cycle or the roles
history: 1.4 | 2026-11-18 | Point 31 of the fix pass: Table 1 marks SASO's editions of ISO 3767-1 and -2 and of ISO 14982 'to confirm', as the register's STD-25 and STD-29; no other change
prepared_by: the functional safety engineer
reviewer: the independent reviewer
approver: the project owner
entity: NASL Technologies (Robotat)
classification: Internal
footer: Shaddad UGV Safety Plan
file: NASL_SAF-001_Shaddad_Safety_Plan_v1.4
outdir: work/reports/safety
chapter_breaks: no
cover_image: none
request: Card G1-07 (CR-002, approved by the owner on 2026-11-16: "Approve option 1"); the independent review of v1.0 (RID-G1-07-01 to -08); the owner's approval of v1.1 and decisions of 2026-11-16 (D79, D80); card G1-01 part 2, step 11: the standards with their editions and the jurisdiction (D103, 2026-11-18)
scope: The safety life cycle of the Shaddad UGV in the mowing phase (D76): the robot as built, its digital twin and the firmware and companion software generated from it, from the requirements (G1) to the safety case (V-05) and the changes after it. Tillage and spraying join the plan when their phases start. Hardware changes are the owner's; this plan sets the safety requirements they must meet.
related: NASL-SHD-HAZ-001 (the hazard log); NASL-SHD-REQ-008 (the system requirements); NASL-SHD-CHR-001 (the charter); NASL-SHD-PMP-001 (the project plan); NASL-EMS-HBK-001 (the engineering handbook); NASL-SHD-CR-001; NASL-SHD-CR-002
-->
# Shaddad UGV: safety plan

## Summary

**The robot works near people, on slopes and in narrow greenhouse aisles.** This plan sets how the project finds its hazards, how safe each safety function must be, who assesses and who approves, and what each gate must show before the robot is used in the field.

- **The life cycle:** five safety cards along the project's gates: the hazard log and this plan (G1-07), the functional safety concept (G2-FSC), the FMEA and fault trees (G3-FME), fault injection (T-06) and the safety case (V-05). Table 4 lists the deliverables at each gate.
- **The jurisdiction and the standards:** the Kingdom of Saudi Arabia, with every standard dated (D103). Table 1 gives each one with the binding Saudi layer in its own column: the SASO machinery regulation and the standards it adopts, the SASO battery regulation, the CST radio specifications and the Labor Law. ISO 12100:2010 for the risk assessment, ISO 18497-1 to -4:2024 for automated agricultural machines, and ISO 13849-1:2023 for the required performance levels (section 3).
- **The rating method:** the risk graph of ISO 13849-1 Annex A, which gives a required level from PL a to PL e (section 4), with project rules for each class. D103 keeps it; ISO 25119 stays the alternative only if a regulation or a contract requires it.
- **Independence:** the functional safety engineer assesses and never designs; the independent reviewer checks every safety record; the project owner approves the plan, accepts the safety goals and the residual risks (section 6).
- **The first result:** the hazard log NASL-SHD-HAZ-001 has 45 hazards for this phase and 12 safety goals, which the owner accepted as the top-level safety requirements (D80): 11 at PL d, with people kept out of the robot's zone in a greenhouse (D79), and one with no PL. None can be shown met on the robot as built; the seven class-1 items are worked on cards G2-FSC and G1-04 (section 10).

## 1. Purpose and scope

The project charter (NASL-SHD-CHR-001, section 3) allows work on the real robot only under an approved procedure and safety plan. This is that plan. It covers:

- the robot as built (`docs/hardware/real_robot_components.md`), its digital twin, and the MCU firmware and companion software generated from the twin;
- every operating mode, place and abnormal case of the charter (section 6): RC driving, autonomous missions, docking and charging, the E-stop with reset and re-arm, and maintenance;
- the safety-related parts of the control system: the E-stop chain and STO, the spring brakes, the person stop, the tilt stop and the descent cap, the stops on a lost link, the watchdog, the blade stop and the battery protection.

It does not cover the tillage and spraying missions of a later phase (their hazards are listed as later-phase entries in the hazard log), product certification (D103 records it as a risk, section 3), or the design of the safety functions, which the subsystem engineers own.

## 2. The safety policy

- **Acceptably safe** means, for this project:
  - every hazard of the hazard log is reduced by the three-step method of ISO 12100: inherently safe design first, then guards and protective devices, then the information for use;
  - every safety goal is met by safety functions at its required performance level (PLr), shown by analysis and by test;
  - the residual risks and the operating limits are listed in the safety case, and the project owner accepts them.
- **The envelope** is part of the safety argument: the 20° design slope with no implement, the mower or the tiller forward; the full-tank sprayer on descents up to about 18.5° (CR-001); the tiller not used until real tiller data exist (the charter, section 3).
- **A gap is never closed by a narrower test.** A goal that a function cannot meet is escalated: a design change, a hardware change, a narrower envelope or an accepted residual risk, each the owner's decision.
- **Until the safety case is accepted** (V-05), the robot is a test article. Section 5.3 sets the rules for work on it.

## 3. The standards, their editions and the jurisdiction

**The jurisdiction is the Kingdom of Saudi Arabia** (the project owner, 2026-11-18, the hazard log's E-07: "Kingdom, dated editions (Recommended)", D103). The sites, the 45-50 °C climate and the fleet service of the specs are there, and no export is planned; the EU rules serve only as the state of the art.

- **The regulators:**
  - the Saudi Standards, Metrology and Quality Organization (SASO), through its SABER platform, for machinery, batteries and low-voltage equipment;
  - the Communications, Space and Technology Commission (CST) for the radio specifications and the frequency licences;
  - the Ministry of Human Resources and Social Development for safety at work (the Labor Law).
- **Every standard is dated.** For each rule and standard, Table 1 gives:
  - its edition;
  - whether it binds in the Kingdom, and through which regulation, or is the state of the art;
  - what it covers here;
  - where the project uses it.

  The editions were checked on 2026-11-18 against the publishers' and the regulators' listings. Those not confirmed say "to confirm".
- **ISO 13849-1:2023 stays the rating method** (section 4), so no hazard is rated again.
- **The machine's classification and its mower standard** are decided on card G2-FSC, with escalation E-06 (D103), because ISO 5395 does not cover a robotic or battery mower.
- **No standards' texts are bought now** (the owner: "Buy none now", D103).
  - The four safety values the standards would set (REQ-SYS-033, -046, -048 and -098) are set by risk assessment and marked as not yet confirmed against the standards.
  - The clause checks that need the texts stay open: the manual-reset clauses of finding F-01, the AgPL bands (section 4, reason 5) and the test object of ISO 18497.
- **Product certification** stays outside this plan (section 1). That is the SASO certificate, the technical file with its risk assessment, and the Arabic labels and manuals. D103 records it as a risk on the risk register.

Table 1. The rules and standards, their editions, whether each binds in the Kingdom, and where the project uses it.

| Rule or standard | Edition | Binding in the Kingdom | What it covers here | Used in |
|---|---|---|---|---|
| SASO Technical Regulation for Machinery Safety, Part 2: Mobile Machinery and Heavy Duty Equipment (TR 01-08-21-180) | approved 08/04/2021, published 21/05/2021, in force 180 days later | yes, a regulation, through SABER: a product certificate (Type 1a, ISO/IEC 17067), a technical file with a declaration and a risk assessment, Arabic labels and instructions | the robot as a mobile machine (HS 8433, powered mowers, or HS 8479, by the classification of G2-FSC); its Annex 1-A adopts the standards marked "through SASO" below, at the latest editions on SASO's site | this plan; the information for use (V-06); certification, a risk (D103) |
| SASO Technical Regulation for Electric Batteries (02-04-18-166) | approved 2018 | yes, a regulation, through SABER; whether it covers a pack built into a machine is to confirm | the pack (SG-10) | G1-04; the risk register |
| GSO technical regulation for low-voltage electrical equipment (BD-142004-01) | its current list (to confirm) | yes, a regulation; it lists IEC 60335-2-29 for battery chargers | the charger (HAZ-031) | G1-04 |
| CST technical specifications for radio equipment: GEN001 (all radio equipment), RI117 (licence-exempt data equipment), RI120 (IMT user equipment) | RI117 and RI120 in force since 20/07/2021; GEN001's issue to confirm | yes, through the equipment approval, which is reported to move to SASO through SABER (SASO 3114:2026; the sources conflict) | the RC set (likely RI117), the LTE modem (RI120), a wireless E-stop (SG-06, SG-07, SG-11) | G1-04, G2-COM, G2-CYB |
| CST frequency licences under the National Frequency Plan | the current plan | yes, for a licensed band | a UHF correction link, if one is fitted | G2-COM |
| The Labor Law (Royal Decree M/51 of 23/08/1426 H, as amended), Articles 122 and 123 | as amended (the ministry's English translation; the Arabic text governs) | yes, on the employer: protection from the hazards of the work and its machines, safety instructions posted in Arabic, workers warned before they start | the use of the robot near workers: the greenhouse rule (D79), the procedures, the information for use | section 5.3; V-06 |
| UN Manual of Tests and Criteria, subsection 38.3 | Rev.8 (2023) with Amendment 1 (2025) | yes, for transport (the dangerous-goods rules) | shipping the pack, or the robot with its pack (SG-10) | G1-04 (the supplier's test summary) |
| Regulation (EU) 2023/1230 on machinery | 2023, applies from 20 January 2027 | no: no EU market is planned | Annex III 3.2.4 (the supervisory function of autonomous mobile machinery) and 3.3 (the safety functions on the machine), as the state of the art | G2-FSC (D103) |
| ISO 12100, risk assessment and risk reduction | 2010 | state of the art | the hazard identification, the risk estimation, the three-step method, the hazard list of Annex B | the hazard log |
| ISO 18497-1 to -4, safety of partially automated, semi-autonomous and autonomous agricultural machinery: design principles (-1, a type-B1 standard), obstacle protection (-2), operating zones (-3), verification and validation (-4) | 2024, all four (they replaced ISO 18497:2018) | state of the art | the hazard list and the goals SG-01, SG-02, SG-03, SG-07 and SG-08 | the hazard log; REQ-SYS-025, -033; G2-FSC; V-05 |
| ISO 13849-1, safety-related parts of control systems: general principles for design | 2023 | state of the art | the rating method (Annex A); the categories, MTTFd, diagnostic coverage, common-cause failures and software; the manual reset; every goal with a PL | section 4; the hazard log; G2-FSC; G3-FME |
| ISO 13849-2, validation | 2012 | state of the art | the validation of the safety functions | V-05 |
| ISO 25119-1 to -4, safety-related parts of control systems of agricultural machinery | -1: 2018 with Amd 1:2020; -2: 2019; -3: 2018 with Amd 1:2020; -4: 2018 with Amd 1:2020 | state of the art (normative in ISO 18497-1) | the AgPL alternative; part 2 excludes lawn and garden equipment | not used for the rating (D103) |
| ISO 13850, emergency stop function | 2015 | state of the art | the E-stop and its reset: the release must not restart the machine (SG-06) | REQ-SYS-063, -064; finding F-01 |
| IEC 60204-1, electrical equipment of machines | 2016 with AMD1:2021 (edition 6.1) | state of the art | the stop categories (D86); the reset of the emergency stop (SG-06) | REQ-SYS-063; the power spec |
| IEC 61800-5-2, adjustable speed drives: functional safety | 2016 (edition 2) | state of the art | STO and SS1 (SG-05, SG-06) | SF-02; G1-04 (the ESC's STO) |
| ISO 14118, prevention of unexpected start-up | 2017 | state of the art | SG-08 | REQ-SYS-001 to -004, -078, -079 |
| ISO 13855, positioning of safeguards with respect to the approach speeds of the human body | 2024 (edition 3; it replaced 2010) | state of the art | sizing the protective fields and zones (SG-01, SG-09) | REQ-SYS-033, -038, -046, -098 |
| ISO 13857, safety distances to keep the upper and lower limbs out of hazard zones | 2019 | state of the art | the deck guard and the blades' reach (SG-09) | REQ-SYS-046, -098; G2-FSC |
| IEC 61496-1 and -3, electro-sensitive protective equipment; part 3, the laser scanners (AOPDDR) | -1: 2020; -3: 2025 (edition 4) | state of the art | a safety-rated person sensor indoors (SG-01, SG-02); part 3 excludes outdoor use | E-02; G2-FSC |
| IEC TS 62998-1, safety-related sensors for the protection of persons | 2019 | state of the art | a safety-rated person sensor outdoors, in the palm groves | E-02; G2-FSC |
| ISO 4254-1, agricultural machinery: safety, general requirements | 2013 with Amd 1:2021 | yes, through SASO (Annex 1-A, row 68; SASO's edition to confirm) | the guards, the controls, hot surfaces, the information for use (SG-12) | SG-12; V-06 |
| The mower standard | to choose on G2-FSC with the machine's classification (D103) | to confirm | the blade guard, the run-down time, thrown objects (SG-09, SG-12). The candidates are IEC 60335-2-107 edition 2.2 (2021), for robotic battery mowers up to 75 V, and ISO 4254-13:2023 with ISO 4254-1, for large rotary mowers (a doubtful fit). ISO 5395-1 to -3:2013 does not apply: it excludes robotic and battery mowers | REQ-SYS-046, -048 and -098, set by risk assessment until then (D103) |
| ISO 5718, blades for rotary mowers | ISO 5718-1:2023 is current; SASO's edition to confirm | yes, through SASO (Annex 1-A, row 118), where it applies | the mower blades; ISO 5718-1 covers the blades of forage disc and drum mowers, so its fit to the deck's blades is decided with the classification | G2-FSC |
| ISO 3767-1 and -2, symbols for the operator's controls and displays | 2016, each with Amd 1:2020 | yes, through SASO (rows 102 and 103; SASO's edition to confirm); normative in ISO 18497-1 | the controls, the console and the labels | V-06 |
| ISO 3600, operator's manuals: content and format | 2022 (edition 5) | yes, through SASO (row 101; SASO's edition to confirm) | the operator's manual (SG-12) | V-06 |
| IEC 62619, secondary lithium cells and batteries for industrial applications | 2022 (edition 2); SASO's annex lists 2017 and asks for the latest | yes, through SASO (row 135) | the pack (SG-10); IEC 60335-2-107, if chosen, points to IEC 62133 instead | G1-04 (the pack's certificate) |
| IEC 60335-2-29, battery chargers | 2016 with A1:2019 | yes, through the GSO low-voltage regulation | the charger (HAZ-031, SG-12) | G1-04 |
| ISO 14982, electromagnetic compatibility of agricultural machinery | 1998 | yes, through SASO (row 89; SASO's edition to confirm) | the robot's EMC | the electronics spec |
| IEC 61000-6-7, immunity of equipment in safety-related systems, industrial locations | 2014 | state of the art | the EMC of the PL d safety functions, until the revised ISO 14982 covers it | G2-FSC; G3-FME |
| IEC 60529, degrees of protection by enclosures (IP code) | edition 2.2 (1989 with Amd 1:1999 and Amd 2:2013) | state of the art | the IP ratings of the specs (the IP65 electronics box, the IP54 charger) | the electronics and power specs |
| ISO 20653, degrees of protection of road vehicles (IP code) | 2023 | state of the art | the IP69K of the LiDAR | the sensors spec |
| A security standard | to choose on G2-CYB: IEC 62443, or ISO 24882 once published | to confirm | the threats to the links (HAZ-043) | G2-CYB |

- **Not used:**
  - IEC 62061:2021 with AMD1:2024 and IEC 61508 (edition 2.0, 2010; edition 3 is near publication), since the project rates with ISO 13849-1;
  - ISO 3691-4, which is for driverless industrial trucks.
- **Checked without a purchase:** clause 2 of ISO 18497-1:2024, in the publisher's free preview, lists ISO 12100:2010, ISO 13849-1:2023, ISO 13849-2:2012, ISO 25119-1 to -4, ISO 4254-1:2013 with Amd 1:2021 and ISO 3767-1 and -2. This closes the check of section 4, reason 1.
- **The watch list.** These editions are being revised now:
  - ISO 12100 and ISO 13849-2 (both at DIS);
  - ISO 13850 (a revision approved in 2026);
  - ISO 25119-1 and -2;
  - IEC 60204-1 and IEC 61496-1;
  - ISO 13857;
  - ISO 14982 (parts 1 and 2 at final draft);
  - IEC 61508 (edition 3) and IEC 62133-2.

  The risk register reviews the list at each gate. A new edition that changes a rating rule or a safety value is brought in through change control (section 5.4), and the hazard log is rated again at its next issue.

## 4. The rating method

**The method.** Each hazardous event of the hazard log is rated with the risk graph of ISO 13849-1:2023 Annex A. Three parameters give the required performance level of the safety function that reduces the risk:

- **S, the severity:** S1 slight and normally reversible; S2 serious and normally irreversible, or death.
- **F, the exposure:** how often and how long people are in the zone that the hazardous event can reach (the robot's path and swept zone, its implement, the zone it can tip into, the air of a fire), while the robot is in that mode and place. F1 is at most once per 15 min and at most 1/20 of the working time, with an exposure reason; F2 is more often or longer. How often the cause occurs (a failure, an emergency, a lost link) never sets F.
- **P, the possibility of avoiding the hazard or limiting the harm:** P1 only where the hazard develops slowly (a motion of at most 0.2 m/s, or a fire that builds over minutes) and the person has room to move away; P2 where the person cannot see it coming, cannot move away (an aisle, a wall, a post, the dock), or the event is sudden.

Table 2. The risk graph: the required performance level PLr.

| Severity | Frequency | P1 | P2 |
|---|---|---|---|
| S1 | F1 | a | b |
| S1 | F2 | b | c |
| S2 | F1 | c | d |
| S2 | F2 | d | e |

The hazard log (section 2) states how the project applies each class, so that two engineers rate a hazard the same way. The rules:

- each event is rated without the safety functions meant to reduce it, warnings and stop commands included: they earn no credit in F or P;
- F is set by the exposure, never by how rare the cause or the event is, and no level is lowered for a low probability of the hazardous event;
- a hazard that arises in several places is rated in the worst place.

**The operating concept sets the exposure.** The owner's greenhouse rule (D79; the charter v1.2, section 6) keeps people out of the robot's zone while it mows a greenhouse aisle; they work in the other aisles. The zone round the robot there is therefore F1, and every safety goal with a PL is at PL d (the hazard log, E-09). People at work beside the robot would set F2 and PL e for six goals, and would need person protection at PL e with safety-rated detection (D79). The log also states the other assumptions behind its exposure classes, for the owner to confirm: few people in a palm-grove row the robot mows, short RC drives, a closed slope test ground, an unattended charging place.

**Why this method and not the ISO 25119 risk graph:**

1. **Both are recognised.** ISO 18497-1:2024 refers normatively to ISO 13849-1:2023 and -2:2012 and to ISO 25119 parts 1 to 4 (its clause 2, read in the publisher's preview, section 3). It is a type-B1 standard, so a machine-specific type-C standard takes precedence where the two differ; card G2-FSC weighs that with the mower standard.
2. **The project already speaks in PL.** The project plan's stakeholder table and the power and electronics specs cite ISO 13849, and they expect PL d for the stop chain (power spec O4, electronics spec 2.5).
3. **The parts are rated in PL.** The parts that will carry the safety functions (a safety relay, the STO of the drives, a certified wireless E-stop, a protective-field sensor) are rated by their makers to ISO 13849-1, IEC 61800-5-2 or IEC 61496. Goals rated in PL let those parts be used as delivered.
4. **Fewer classes to justify.** The ISO 13849-1 graph has two classes per parameter. ISO 25119-2 rates severity, exposure and controllability in finer classes, which need field data on the exposure that the project does not have yet.
5. **The levels carry across.** ISO 25119 uses five levels, AgPL a to e, defined by the same bands of the average probability of a dangerous failure per hour as PL a to e (not yet confirmed against the texts, which are not bought now, D103). If a regulation or a contract requires ISO 25119, each hazard is rated again with its graph; the parameters recorded in the log map across.

**The achieved level** of each safety function is shown later, with the methods of ISO 13849-1: the category of its architecture, the mean time to a dangerous failure (MTTFd) of its channels, the diagnostic coverage, the measures against common-cause failures and the software measures (G2-FSC sets them, G3-FME computes them).

## 5. The safety life cycle

### 5.1 The phases and the gates

The safety life cycle follows the project's tailored gates (NASL-SHD-PMP-001; NASL-EMS-HBK-001). Each safety deliverable is a controlled document in `docs/safety/` with its PDF, checked by the independent reviewer before its gate.

Table 3. The safety life cycle on the project's gates.

| Phase | Gate | Safety activity | Card |
|---|---|---|---|
| Concept and requirements | G1 (SRR + SFR) | the hazard analysis and risk assessment; the safety goals; this plan | G1-07 |
| Concept and requirements | G1 | the safety goals into the requirements baseline as top-level safety requirements; the standards' editions and the jurisdiction confirmed | G1-01, checked at G1-05 |
| Preliminary design | G2 (PDR) | the functional safety concept: for each goal its safety functions, safe state, reaction time, PLr, architecture, allocation and verification; the safety requirements in the register | G2-FSC |
| Preliminary design | G2 | the descent cap and the sprayer envelope; a stop without skidding (proposed); the threats to the links (proposed) | G2-DSC, G2-BRK, G2-CYB |
| Detailed design | G3 (CDR) | the FMEA and fault trees of the safety chain; the category, MTTFd, diagnostic coverage and common-cause measures of each function; the failure modes for fault injection | G3-FME |
| Detailed design | G3 | the designs of the safety-relevant subsystems to the safety requirements | G3-MCU, G3-PWR, G3-DRV, G3-NAV, G3-GNC |
| Build | Build | the safety-related firmware generated with the rules of section 5.2; the embedded toolchain | B-00, B-01 |
| Verification | G4 (TRR) | fault injection in the system model: each detected failure reaches its safe state in time | T-05, T-06 |
| Verification | G5 (FCA / SVR) | the verification campaigns in simulation, on the bench and in the field; the safety case, the residual risks and the operating limits | V-01, V-02, V-03, V-05 |
| Operation | G5 and after | the operator's and the maintainer's manuals with the safety information; the hazard log kept current through every change | V-06 (proposed), change control |

Table 4. The safety deliverables at each gate.

| Gate | Deliverable | Document | Gate criterion | Checked by | Approved by |
|---|---|---|---|---|---|
| G1 | The safety plan | NASL-SHD-SAF-001 (this document) | G1-SC9 | the independent reviewer | the project owner (approves) |
| G1 | The hazard log and the safety goals | NASL-SHD-HAZ-001; `docs/safety/hazard_log.json` and its Safety Analysis Manager file | G1-SC9: every mode and abnormal case of the charter covered | the independent reviewer | the project owner (accepts the goals) |
| G1 | The goals as top-level safety requirements | the requirements baseline (G1-01) | G1-01 exit | the independent reviewer | the project owner |
| G2 | The functional safety concept and the safety requirements | NASL-SHD-FSC-001; the requirements register | G2-SC8: 100 % of the goals allocated | the independent reviewer | the project owner |
| G3 | The FMEA and fault trees; the achieved levels | NASL-SHD-FMA-001 and its Safety Analysis Manager files | G3-SC7: every gap closed or on a card | the independent reviewer | the project owner |
| G4 | The fault-injection test report | the T-06 report | T-06 exit: every detected failure mode injected, safe state in time | the independent reviewer | the project owner |
| G5 | The safety case: the argument, the evidence, the residual risks, the operating limits | NASL-SHD-SCS-001 | G5-SC5: assessed, no Critical or Major finding open | the independent assessment | the project owner (accepts the residual risks) |

### 5.2 Safety-related software

The MCU firmware and the companion software are generated from the twin. Where a safety function runs in software, its software is safety-related software in the sense of ISO 13849-1 (clause 4.6). The concept (G2-FSC) sets, for each such function at its level:

- the specification of the safety requirements of the software and their trace to the models;
- the modelling and coding rules, checked in the models and on the generated code;
- the reviews, the tests and their coverage, and the independence of the review;
- the configuration and change control of the models, the code and the tools (version control comes with card G1-08);
- what the tools must show to be trusted for the job.

### 5.3 Work on the real robot before the safety case

Until the safety case is accepted for a use, the robot is a test article:

- **Only the project owner acts on the real robot** (the charter, section 3), under a test procedure the owner approves.
- **Every procedure names its hazards** from the hazard log and the measures that hold during the test, among them how the robot is stopped in an emergency and who can reach that means. The functional safety engineer reviews each procedure before the owner approves it.
- **No one else is in the operating zone** during a test unless the procedure allows it and names the measure that protects them.
- **The envelope holds** in every test: the CR-001 limits; the tiller not used.
- **Each test that touches a safety function** (the slope brake test T-04, the field campaign V-03) records its result as evidence for the safety case.

### 5.4 Changes and the upkeep of the hazard log

- **Every change request** names its safety impact (the impact table of the change request). The functional safety engineer assesses it against the hazard log before the change board decides.
- **Every nonconformance, field event and review finding** that touches a hazard is checked against the log; a new hazard gets the next free number.
- **Hazards are never renumbered.** A closed hazard stays in the log with its closure evidence.
- **The log is rated again** when the concept, the supplier data or a change alters a cause or a measure, and at each gate.

## 6. The independence of the assessment

- **The functional safety engineer assesses and never designs.** The engineer sets the required levels, checks that each safety requirement is met and verified, and escalates a gap, but never designs or changes a safety function: the subsystem engineers do (CR-002 Appendix D). That keeps the assessment independent of the design.
- **The independent reviewer checks every safety record** (this plan, the hazard log, the concept, the FMEA and fault trees, the fault-injection report, the safety case) and every change to a safety function. The reviewer works read-only, from the records and the evidence, without the engineer's reasoning, and returns ranked findings with a verdict. A safety record goes to the owner only with no Critical or Major finding open.
- **The project owner approves:** this plan, the safety goals, the residual risks and every decision on the hardware, the envelope and the operating concept.
- **Validation by persons independent of the design**, as ISO 13849-2 asks for, is given by the functional safety engineer and the independent reviewer, neither of whom designs the functions.
- **An external assessment** of the safety case, by a body independent of the project, is proposed before the robot works with people other than the test team in its operating zone. That is the owner's decision (section 10).

## 7. The roles

Table 5. The roles in the safety life cycle.

| Role | In the safety life cycle | Decides |
|---|---|---|
| The project owner | sets the safety policy; approves this plan; accepts the safety goals and the residual risks; decides the hardware, the suppliers, the envelope and the operating concept; the only person who acts on the real robot | every approval |
| The functional safety engineer | leads the life cycle: this plan, the hazard log, the concept, the FMEA and fault trees, the safety case; sets the required levels; assesses every change and every test procedure on the robot for safety; escalates gaps | the required levels, for the owner's acceptance |
| The independent reviewer | checks every safety record and every change to a safety function, read-only; ranks the findings; gives the verdict | the verdict on each record |
| The lead systems engineer | takes the safety goals into the requirements baseline (G1-01); runs the change control, the registers and the gates; assigns the work | the plan of the work |
| The subsystem engineers (structure; power and BMS; drive; MCU firmware; electronics and comms; sensors and navigation; GNC) | design and change the safety functions of their subsystems to the safety requirements; supply the hazards of their subsystems and the data for the FMEA | their designs, within the requirements |
| The integration engineer | leads the fault-injection campaign (T-06) and the system scenarios; checks the top level after each change | none |
| The test engineer | runs the test campaigns and sorts their results | none |
| The configuration and documentation engineer | keeps the safety records under configuration control and in the document register | none |
| The technical writer | writes the safety information of the operator's and the maintainer's manuals (V-06, proposed) | none |
| The operator and the maintainer | follow the procedures and the operating limits; report every event that touches safety | none |

## 8. The safety records

- **Where:** `docs/safety/`, each record a controlled document with a source, a PDF built with `tools/reports/build_document.py`, a version and a history line.
- **The hazard log** is kept as a structured record, `docs/safety/hazard_log.json`, from which the document NASL-SHD-HAZ-001 is generated (`docs/safety/hazard_log_render.py`, whose `--check` verifies the levels, the traces and the walk-through). It is to be mirrored in the Safety Analysis Manager (`docs/safety/hazard_log.mldatx`) once the licence allows it (open point OP-01, section 10). The FMEA and the fault trees of G3-FME are Safety Analysis Manager files with their PDF.
- **The planned records:** NASL-SHD-SAF-001 (this plan), NASL-SHD-HAZ-001 (the hazard log), NASL-SHD-FSC-001 (the concept), NASL-SHD-FMA-001 (the FMEA and fault trees), NASL-SHD-SCS-001 (the safety case), as the document register lists them.
- **Review records:** the independent reviewer's findings on each record are kept with it, and the owner's approval is recorded word for word in the decision log.

## 9. The verification and validation of safety

Each safety requirement is verified by one or more of these methods, chosen in the concept:

- **Analysis:** the FMEA and fault trees, the category, MTTFd, diagnostic coverage and common-cause measures (G3-FME).
- **Simulation:** the system scenarios that exercise the abnormal cases (the stop on a slope S2, the person in the path S5, lost RC S6, lost GNSS S7, the E-stop S8), and fault injection with Simulink Fault Analyzer (T-06). A claim made from simulation is bounded by the credibility level that card G2-VAL gives the model, and by the estimated parameters it rests on.
- **Bench:** the safety chain, the drives and the brakes on the bench (V-02).
- **Field:** the slope brake test (T-04) and the field campaign (V-03), under the rules of section 5.3.
- **Inspection:** the independent reviewer's checks of the records and the designs.

The validation of each safety function follows ISO 13849-2: its specification is checked against the goal, its design against the specification, and its level against the PLr.

## 10. Competence, open points and decisions

- **Approval:** the project owner approved this plan (v1.1) and accepted the 12 safety goals of the hazard log as the top-level safety requirements on 2026-11-16, card G1-07: "Approve and accept (Recommended)" (D80). Issue 1.2 recorded the owner's decisions. Issue 1.3 adds the standards with their editions and the jurisdiction (D103, section 3); it goes to the independent review of G1-01 part 2 and to the owner with the baseline.
- **Competence:** each safety role is held by an engineer who knows the standards of section 3. The owner confirms the competence of the functional safety engineer and of the independent reviewer at G1-05.
- **The owner's decisions of 2026-11-16:**
  - the rating method of section 4: approved with this plan (D80) and kept by D103; ISO 25119 stays the alternative only if a regulation or a contract requires it;
  - the greenhouse operating concept (E-09 of the hazard log): "Keep people out (Recommended)" (D79). People are kept out of the robot's zone while it mows a greenhouse aisle and work in the other aisles, so every goal with a PL is at PL d;
  - which safety devices the robot actually has (the question of E-01): "E-stop button(s), Safety relay / STO wiring" (D80). Their data, category and wiring come from the supplier on card G1-04;
  - the hardware the safety goals need (the class-1 escalations E-01 to E-06 and E-08 of the hazard log, section 7): worked on card G2-FSC, whose exit asks for each one the architecture that meets its goal at the level D79 and D80 set, or a change request for new hardware, with the supplier data of card G1-04. Nothing is bought now (D80).
- **The owner's decision of 2026-11-18:** the editions and the jurisdiction (E-07 of the hazard log): "Kingdom, dated editions (Recommended)"; on the standards' texts: "Buy none now" (D103). Section 3 and Table 1 record them.
- **Open points of D103:**
  - the machine's classification and its mower standard: card G2-FSC, with E-06;
  - the binding layer's unknowns, for the project owner through a SASO conformity body: the robot's tariff code and so the regulation that covers it; the editions SASO adopted; who places the robot on the market and holds the certificates; whether the battery regulation covers a built-in pack; the radios' approval route;
  - the clause checks that need the standards' texts (section 3).
- **Decision still open for the project owner:** whether the safety case gets an external assessment before the robot works with other people in its zone.
- **Open point OP-01, the Safety Analysis Manager copy of the hazard log.** Simulink Fault Analyzer is installed but not licensed to the project's MATLAB (its own licence check, 2026-11-16). The project owner decides on the licence by 2026-11-23; the functional safety engineer then makes the copy from the record in one batch run, before G1-05. Until then the record and its PDF are the hazard log. The FMEA and fault trees of G3-FME need the same licence.

## References

- NASL-SHD-HAZ-001, the hazard log (`docs/safety/hazard_log.md`, record `docs/safety/hazard_log.json`).
- NASL-SHD-REQ-008 v1.0, the system requirements specification (the safety goals as top-level safety requirements).
- NASL-SHD-CHR-001 v1.3, the project charter; NASL-SHD-PMP-001 v1.4, the project plan; NASL-EMS-HBK-001, the engineering handbook.
- NASL-SHD-CR-001 v1.2, the descent envelope of the full-tank sprayer; NASL-SHD-CR-002 v1.1, the plan after G0 (Appendix A.1 and D).
- The task cards, `tools/dashboard/project_cards.json`: G1-07, G1-01, G2-FSC, G3-FME, T-06, V-05, G2-DSC, G2-BRK, G2-CYB.
- The as-built record `docs/hardware/real_robot_components.md`; the specs `docs/specs/*.md`; the decisions D30 to D103 in `docs/architecture.md`.
- The rules and standards of Table 1, at the editions given there (D103).
