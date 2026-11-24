<!--
doc_no: NASL-<CODE>-TP-<NNN>
doc_type: TEST PROCEDURE
title: <Project>: <test name>
subtitle: <campaign: simulation / bench / field>
summary: <what the test shows and which requirements it verifies>
version: 1.0
date: <YYYY-MM-DD>
status: Issued for approval
history: 1.0 | <YYYY-MM-DD> | First issue
prepared_by: <test engineer>
reviewer: [TBD]
approver: <the project owner: approves every procedure before G4>
classification: Internal
footer: <Project> TP-<NNN>
file: NASL_TP-<NNN>_<Project>_<Test_Name>_v1.0
outdir: work/reports/tests
-->
# <Test name>

## 1. Purpose and the requirements verified

Table 1. The requirements this test verifies.

| Requirement | Text (short) | Pass criterion |
|---|---|---|
| REQ-<DOM>-NNN | <...> | <measured value and its threshold, with units> |

## 2. Test article and configuration

<Item, versions (models, firmware, hardware), and the baseline it must match.>

## 3. Equipment and environment

<Tools, instruments (calibration), site, weather limits, data recording.>

## 4. Safety

<Hazards, precautions, stop rules: who may stop the test and when; the safety plan reference. Hardware and field tests are run by the owner or under the owner's control.>

## 5. Steps

1. <action> - <expected observation> - <data recorded>
2. <action> - <expected observation> - <data recorded>

## 6. Pass and fail

<The rule per requirement; what counts as an invalid run; what is repeated.>

## 7. Data and the report

<The files recorded and where they go; the test report template is framework/templates/test_report.md.>
