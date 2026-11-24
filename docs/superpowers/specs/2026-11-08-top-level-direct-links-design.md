# Direct links at the top level of Shaddad_System

Short design, 2026-11-08. For the top level only, it replaces section 5 of
`2026-10-29-model-layout-cleanup-design.md` (option A, one system bus). The user's sketch:
`2026-11-08-top-level-interconnected-sketch.png`.

## Intent

The top level today is one column of 11 blocks feeding one `System_Bus`. It has no crossings, but it doesn't show who
feeds whom. The user wants a block diagram instead: each subsystem its own block, joined directly to the blocks it
exchanges signals with, so the picture itself shows the connections.

## Goals

- Today's 11 blocks stay, each its own block (the user's choice, 2026-11-08).
- `System_Bus` goes. Each link between two blocks is one named bus line, from the producer's output to an input port of
  the consumer.
- A block's input ports are named after the blocks that feed it, so its ports list its sources.
- The blocks are spread in two dimensions as in the sketch, with the most connected blocks in the middle.

## Non-goals

- Plant stays one block; Power, Drive, Structure and Electronics stay inside it.
- Nothing inside a block changes except the port names of its pick blocks.
- No other diagram or file is laid out again.

## Anti-goals

- Any change in what the model computes.
- Goto/From tags (layout spec L6), lines behind blocks, overlapping blocks or labels.

## Constraints

- Layout spec 8's targets hold for this diagram: 0 overlaps, lines through blocks, unnamed lines, unconnected ports and
  line overlaps; every crossing listed in the allowance with its reason. Rule L1 (left to right) is relaxed for this one
  diagram, because the links run both ways.
- The Model blocks' own ports, the ICD's signals, the logged signals and their paths stay as they are.
- MATLAB in separate batch processes, one heavy run at a time, on the same rules as the layout plan.

## Approaches

- **A: direct lines, one input port per source (the user's choice).** The closest match to the sketch, and the ports
  list the sources. A new dependency adds a port and a line at the top level.
- B: one input bar per block. Each block keeps one input, `sys`, with a Bus Creator in front of it. Nothing inside
  changes, but it adds 11 bars and a busier picture.
- Dropped: placing the blocks with the layout engine, left to right. It wouldn't look like the sketch, and the seven
  two-way pairs would send long feedback lines under everything.

## The design

**Links, measured from the saved model: 28 between blocks, and the scenario input to 9 blocks.**

| Block | Output bus | Input ports (one line each) |
|---|---|---|
| Operator_Console | `operator` | `mcu`, `plant`, `scen` |
| Operator_RC | `rc` | `operator`, `scen` |
| MCU | `mcu` | `rc`, `can1`, `can2`, `plant`, `sensors`, `scen` |
| BMS | `bms` | `can1`, `plant` |
| CAN1_Net | `can1` | `mcu`, `bms`, `plant`, `scen` |
| Plant | `plant` | `mcu`, `bms`, `can1`, `scen` |
| Sensors | `sensors` | `plant`, `scen` |
| CAN2_Link | `can2` | `mcu`, `gnc`, `plant`, `scen` |
| GNC | `gnc` | `can2`, `sensors`, `scen` |
| Viz_Logging | none | `mcu`, `gnc`, `plant` |
| Sim_Monitors | `monitors` | `mcu`, `gnc`, `plant`, `scen` |

**Ports and lines.**
- Each block keeps its one output bus, as today. Each input is an In Bus Element port named after its source; a pick that
  is `sys.plant.truth.pose2D` today becomes element `truth.pose2D` of port `plant`, in a block named `plant_truth_pose2D`.
- Each line carries the producer's whole output bus and is named after it. The buses stay virtual.
- The scenario input runs as one line named `scen`, a trunk with a branch to each of the 9 blocks that read it.
- Nothing reads `monitors` today; it only fed `System_Bus`. It ends in one Terminator beside the block, so the output
  stays and no port is left unconnected.

**Placement** (set by the builder, kept by the layout engine, which still adds and checks the lines):

```
                CAN2_Link     Operator_RC   Operator_Console
GNC             Sensors       MCU           CAN1_Net
Sim_Monitors    Viz_Logging   Plant         BMS
```

A straight-line estimate gives about 7 crossings between the 28 links; the scenario trunk adds some. Seven pairs feed
each other both ways (MCU-Plant, MCU-CAN1, MCU-CAN2, GNC-CAN2, BMS-CAN1, BMS-Plant, CAN1-Plant), so some crossings stay.

**What changes:**
- `scripts/build/build_shaddad_system.m`: the pick helper, the wiring (no `System_Bus`) and the placement;
- `tests/integration/t_layout.m`: `testTopStructure` for the new structure, and any other top-level test it affects;
- `utilities/shaddad_layout_allowance.m`: a row for the top level's crossings, with the reason;
- the docs that describe the top level: `docs/icd_static_sections.md` (then regenerate `docs/icd.md`) and the model map in
  `docs/STATUS.md`; a pointer from section 5 of the layout spec to this file. `docs/report/**` is not edited; its stale
  figures are listed for the user.

**Proof that nothing else changed.** After `build_shaddad_system()`:
- the layout check of the top level, and `t_layout` tag Layout;
- `shaddad_check_interfaces('Compiled', true)`: 0; algebraic loops: 0;
- the variant sweeps, both passes, against the same baselines: no new failures (the console's variant choices sit on the
  links that change);
- the 8 compare runs against the same baselines: all bit-identical, warnings as before;
- `run_regression('Fast')`: only its known failures.

## Left to the builder

The exact positions, block sizes, the order of each block's input ports, and the route of the scenario trunk, chosen to
keep crossings low. A picture of the real result goes to the user before it is accepted.
