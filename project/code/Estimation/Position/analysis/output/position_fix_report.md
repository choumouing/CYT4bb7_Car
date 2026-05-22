# Position Beacon Fix Offline Report

- Log directory: `D:\HDUASC-SmartCar-21st-FlyOverMinefield\CYT4bb7_Car\project\code\Estimation\Position\惯导结合信标检测矫正`
- CSV logs scanned: 18
- Overall pass: 18/18
- No-beacon rejection pass: 5/5
- Beacon sequence pass: 13/13
- Final error <= 0.50m: 18/18

## Parameters

- EVENT_STRICT_RADIUS_M = 0.35
- EVENT_LOOSE_RADIUS_M = 0.9
- EVENT_AMBIGUITY_MARGIN_M = 0.12
- EVENT_MIN_SCORE = 0.5
- EVENT_MIN_CONFIDENCE = 1.0
- TRACK_DIAG_RADIUS_M = 0.12
- ODOMETER_REPLAY_SCALE_X = 1.0
- ODOMETER_REPLAY_SCALE_Y = 1.06
- INITIAL_TRACK_BEACON7_RADIUS_M = 0.07
- INITIAL_TRACK_BEACON7_MIN_SCORE = 0.6
- FIX_ALPHA_INITIAL = 0.75
- FIX_ALPHA_TRANSITION = 0.5
- FIX_ALPHA_TRACK = 0.0
- FIX_ALPHA_WEAK = 0.0
- FIX_ALPHA_STRICT = 0.75
- FIX_ALPHA_REPEAT_ENTER = 0.5

## Per-log Summary

| File | Expected | Baseline | Offline | Truth | Offline final | Error/m | Status | Notes |
| --- | --- | --- | --- | --- | --- | ---: | --- | --- |
| 实际上没有碰到信标灯-最终坐标1,4.csv | - | 3 | - | (1.00,4.00) | (1.110,3.619) | 0.396 | PASS |  |
| 实际上没有碰到信标灯-最终坐标3,3(第二次).csv | - | - | - | (3.00,3.00) | (3.269,2.820) | 0.324 | PASS |  |
| 实际上没有碰到信标灯-最终坐标3,3.csv | - | - | - | (3.00,3.00) | (2.990,2.827) | 0.173 | PASS |  |
| 实际上没有碰到信标灯-最终坐标5,2.csv | - | - | - | (5.00,2.00) | (5.067,1.927) | 0.099 | PASS |  |
| 慢速-实际上没有碰到信标灯-最终坐标5,3.csv | - | - | - | (5.00,3.00) | (5.013,2.843) | 0.158 | PASS |  |
| 经过2,3号信标灯-最终坐标1,3.csv | 2,3 | - | 2,3 | (1.00,3.00) | (1.125,3.129) | 0.179 | PASS |  |
| 经过3号信标灯-最终坐标3,2.csv | 3 | - | 3 | (3.00,2.00) | (2.852,1.855) | 0.207 | PASS |  |
| 经过4,3号信标灯-最终坐标0,3.csv | 4,3 | 3 | 4,3 | (0.00,3.00) | (0.162,2.989) | 0.162 | PASS |  |
| 经过4号信标灯-最终坐标1,3.csv | 4 | - | 4 | (1.00,3.00) | (0.976,2.894) | 0.108 | PASS |  |
| 经过6,3号信标灯-最终坐标5,2.csv | 6,3 | 6,3 | 6,3 | (5.00,2.00) | (4.727,1.987) | 0.273 | PASS |  |
| 经过6,3号信标灯-最终坐标5,3.csv | 6,3 | 6,3 | 6,3 | (5.00,3.00) | (4.976,2.745) | 0.256 | PASS |  |
| 经过6,4,3号信标灯-最终坐标1,4.csv | 6,4,3 | 6,3 | 6,4,3 | (1.00,4.00) | (0.928,4.068) | 0.099 | PASS |  |
| 经过6,4,3号信标灯-最终坐标3,2.csv | 6,4,3 | 3 | 6,4,3 | (3.00,2.00) | (3.259,1.780) | 0.340 | PASS |  |
| 经过6,4号信标灯-最终坐标0,2.csv | 6,4 | 6 | 6,4 | (0.00,2.00) | (0.168,2.111) | 0.202 | PASS |  |
| 经过7,6,4号信标灯-最终坐标2,1.csv | 7,6,4 | 7 | 7,6,4 | (2.00,1.00) | (1.510,0.941) | 0.493 | PASS |  |
| 经过7655443267信标-最终坐标5,2.csv | 7,6,5,5,4,4,3,2,6,7 | 7,6,5,5,4,4,2,7 | 7,6,5,5,4,4,3,2,6,7 | (5.00,2.00) | (4.853,2.079) | 0.167 | PASS |  |
| 经过7号信标灯-最终坐标3,2.csv | 7 | 7 | 7 | (3.00,2.00) | (2.777,1.814) | 0.291 | PASS |  |
| 经过7号信标灯-最终坐标4,1.csv | 7 | - | 7 | (4.00,1.00) | (4.061,0.983) | 0.063 | PASS |  |

## Rejections And Diagnostics

Rejected detector events and trajectory-only diagnostics are written to `position_fix_rejections.csv`. A `track` row means the inertial path passed near a beacon coordinate but no accepted detector event confirmed it; it is diagnostic evidence, not an applied fix.

Accepted events are written to `position_fix_events.csv`. The `counts_in_sequence` column is the authoritative sequence gate: repeat calibration events may be applied to position while staying out of the recognized beacon sequence.

## Manual Review Items

- None.

## Scale Sweep

A coarse velocity-scale sweep was run for x/y scales 0.94..1.06 step 0.02. This diagnostic checks whether a simple odometer scale change can satisfy the same sequence and final-error gates.
- Best scale: x=1.00, y=1.06
- Best pass count: 18/18
- Failed files at best scale: none


## C Port Gate

The offline matcher is the required gate before changing `fixator.c`. If this report is not fully passing, the current rules must not be ported to C yet. Failed rows need better detector evidence, additional logs, or a separate odometer calibration pass before the embedded fixator can be changed safely.

Do not directly map every applied offline event to the current 28-channel C log as `fix_applied + beacon_index`. The offline model distinguishes position calibration from sequence recognition with `counts_in_sequence`; the current C telemetry does not expose that bit. Porting the repeat-enter calibration without extending diagnostics would make a valid calibration look like an extra beacon in sequence extraction.

## Verdict

All offline acceptance gates passed for the simulated matcher.
