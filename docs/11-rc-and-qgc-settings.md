# 11 — RC and QGC Settings

The configuration of the Holybro X500 V2 as it stands after bench bring-up
([10](10-hardware-bringup.md)) and the first flights ([12](12-first-flights-2026-10-28.md)):
the RadioMaster Pocket (EdgeTX, ELRS) and the PX4 parameters set through QGroundControl.

---

## Overview

Holybro X500 V2 with a Pixhawk 6C on PX4 v1.17.0, built as the custom `px4_fmu-v6c_crsf`
variant so the `crsf_rc` driver can read the ExpressLRS receiver. Takeoff weight 1.69 kg with
battery; stock motors and props.

| Device | Pixhawk 6C port | Notes |
|---|---|---|
| ELRS RP3 receiver | **TELEM3** (`/dev/ttyS1`) | RP3 TX → FC RX, RP3 RX → FC TX (wires were swapped once to get data flowing) |
| SiK 915 MHz air radio | TELEM1 | MAVLink to QGC at 57600 baud |
| M10 GPS (compass, safety switch) | GPS1 | Mast arrow forward |
| PM02 power module | POWER1 | 4S LiPo |
| ESC signals | FMU or I/O PWM OUT | Motors verified for position and direction |

Radio: RadioMaster Pocket (internal ELRS 2.4 GHz, EdgeTX), ELRS 3.3.1 on the transmitter, RP3
on 3.x. Bound with the bind button, no binding phrase.

## EdgeTX (RadioMaster Pocket)

One mix line per channel, weight 100, no `+=` lines. Arm, Return and kill are 2-position or
far-end switches so nothing flips by accident.

**Model setup and ELRS module** (MDL → MODEL SETUP; SYS → Tools → ExpressLRS)

| Setting | Value |
|---|---|
| Internal RF | CRSF |
| Packet rate | 250 Hz (150 Hz for more range) |
| Telemetry ratio | 1:16 |
| Switch mode | Hybrid (CH6–12 low resolution: switches only) |
| TX power | Dynamic, or a fixed 100 mW |

**Mixes** (MDL → MIXES)

| Channel | Source | Function in PX4 |
|---|---|---|
| CH1 | Ail (right stick left/right) | Roll |
| CH2 | Ele (right stick up/down) | Pitch |
| CH3 | Thr (left stick up/down) | Throttle |
| CH4 | Rud (left stick left/right) | Yaw |
| CH5 | SA (2-pos) | Arm |
| CH6 | SB (3-pos) | Flight mode |
| CH7 | SD (2-pos) | Return |
| CH8 | SC (3-pos) | Kill, far end only |

SE (momentary) is unused: a momentary kill switch releases the kill when let go.

The Pocket's throttle stick does not spring back to centre. In Position and Altitude mode
centre means "hold height" and the bottom means "descend at 1.5 m/s", so return it to the middle
by hand.

**Telemetry** (MDL → TELEMETRY → Discover new sensors): RQly, 1RSS/2RSS, TQly/TRSS, RxBt, Bat%.
On the bench RQly should read about 100 and 1RSS roughly −20 to −50 dBm. A low-signal warning
just at power-up, before the link forms, is normal.

## QGroundControl / PX4

Airframe is Quadrotor x → Holybro X500 V2 (4019). Gyro, accelerometer and level horizon are
calibrated on the bench; the compass is calibrated outdoors with everything in its final position
(redo it when the Jetson goes on).

**RC input (CRSF)**

| Parameter | Value |
|---|---|
| `RC_CRSF_PRT_CFG` | TELEM 3 |
| `RC_CRSF_TEL_EN` | 1 (Enabled) |
| `MAV_1_CONFIG` | Disabled, if it was on the CRSF port |
| `COM_RC_IN_MODE` | 0 (RC only) |

Check from the MAVLink console with `crsf_rc status`: the UART device should be `/dev/ttyS1`
(TELEM3), and UART RX bytes and Valid known packet CRCs should climb.

**Flight modes and switches** (Vehicle Setup → Flight Modes)

| Function | Channel | Positions |
|---|---|---|
| Mode channel | 6 (SB) | Position / Altitude / Stabilized, Position at the easiest-to-reach end |
| Arm switch | 5 (SA) | |
| Return switch | 7 (SD) | |
| Kill switch | 8 (SC) | `RC_MAP_KILL_SW` = 8 |

PX4 only acts on a mode switch when SB changes position. To take control back from an
autonomous or offboard mode while SB already sits on Position, flip it to Altitude (or out and
back).

**Battery** (Vehicle Setup → Battery)

| Parameter | Value | Notes |
|---|---|---|
| `BAT1_SOURCE` | Power Module (analog) | |
| `BAT1_N_CELLS` | 4 | |
| `BAT1_V_EMPTY` / `BAT1_V_CHARGED` | 3.5 V / 4.2 V | Per cell |
| `BAT1_V_DIV` | 18.478 | Trimmed to match a multimeter on the pack |
| `BAT1_A_PER_V` | **53.6** | Was 36.37; PX4 read current 1.47× low (three charger comparisons, [12](12-first-flights-2026-10-28.md)) |
| `BAT1_CAPACITY` | **3700 mAh** | Was 5200 (the label); both packs hold about 3700 |
| `BAT1_R_INTERNAL` | **0.004 Ω** per cell | Was −1 (auto), whose estimate fell to 0 in flight, so the battery % followed the sagging voltage |

Check after the next flight from a full charge: QGC's mAh used should be within about 5 % of
what the charger puts back.

**Actuators**: Motors 1–4 assigned to the ESC outputs, DShot600 on FMU PWM OUT or PWM 400 Hz
on I/O PWM OUT. Verified order and spin: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW,
4 rear-right CW.

**Safety**

| Parameter | Value | Effect |
|---|---|---|
| `NAV_RCL_ACT` | 2 | RC loss → Return |
| `COM_RC_LOSS_T` | 0.5 s | Time without RC before it counts as lost |
| `NAV_DLL_ACT` | 0 | SiK telemetry loss does nothing |
| `COM_LOW_BAT_ACT` | 3 | Return at critical, land at emergency |
| `BAT_LOW_THR` | 0.25 | Warning |
| `BAT_CRIT_THR` | 0.15 | Return |
| `BAT_EMERGEN_THR` | 0.07 | Land where it is |
| `GF_ACTION` | 3 | Geofence breach → Return |
| `GF_MAX_HOR_DIST` | 100 m | |
| `GF_MAX_VER_DIST` | 40 m | |
| `RTL_RETURN_ALT` | 20 m | Above the trees at the field |
| `MPC_VEL_MANUAL` | 4 m/s | Gentler Position mode while learning |
| `COM_RC_OVERRIDE` | **0** | Was 1. Sticks can no longer cancel Return, Land or failsafes; take control with the mode switch. In v1.17 override triggers on stick *deflection*, so the non-centring throttle cancelled a geofence Return before the crash ([12](12-first-flights-2026-10-28.md)). |

`COM_RC_OVERRIDE` and the Jetson: bit 1 (value 2) would allow stick takeover in offboard mode
(`velocity_offboard_control.py`). ROS 2 custom modes (`px4_missions`, `waypoint_mission.py`) are
external modes, neither auto nor offboard, and ignore stick override at any value. The mode
switch works in all of them. PX4 1.18 replaces both `COM_RC_OVERRIDE` and `COM_RC_STICK_OV` with
`MAN_OVERRIDE_SPD`, which triggers on stick speed instead.
