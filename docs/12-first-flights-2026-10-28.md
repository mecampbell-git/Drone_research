# 12 — First Flights (2026-10-28)

The first outdoor flights of the X500 V2, milestone **M1** in [06](06-autonomy-roadmap.md): two
sessions, about 30 minutes in the air on four battery charges, flown in Position mode. The aircraft
flew well. One flight ended in a tree, caused by piloting plus a stick-override setting, not by a
fault; the logs also showed that PX4 misreads battery current and that the packs hold less than
their label. Settings changed as a result are in [11](11-rc-and-qgc-settings.md).

---

## Flights

Times are local (UTC−7); the SD-card log names are UTC. Heights and distances are from the home
(takeoff) position. mAh is PX4's count since power-up, so it accumulates over flights on one pack,
and it reads 1.47× low (see Battery).

| SD log | QGC log | Local | Pack | Airborne | Max dist | Max height | mAh (cumulative) | How it ended |
|---|---|---|---|---|---|---|---|---|
| `16_20_56` | 9 | 09:21 | 1 | ground only | | | 1 | Armed and disarmed |
| `16_21_11` | 10 | 09:23 | 1 | 113 s | 41 m | 19 m | 389 | Return switch |
| `16_23_35` | 11 | 09:28 | 1 | 251 s | 96 m | 38 m | 1279 | Return switch |
| `16_30_42` | 12 | 09:31 | 1 | ground only | | | 1280 | Armed and disarmed |
| `16_30_56` | 13 | 09:34 | 1 | 198 s | 83 m | 29 m | 2003 | Critical battery → Return |
| `16_36_24` | 14 | 09:39 | 2 | 171 s | 105 m | 27 m | 589 | **Crash into a tree** |
| `20_29_48` | | 13:29 | A | 495 s | 55 m | 15 m | 1811 | Return switch (critical battery 2 s later) |
| `20_39_08` | | 13:39 | A | 23 s | 0 m | 5 m | 1894 | Return switch |
| `20_41_53` | | 13:41 | B | ground only | | | 0 | Armed and disarmed |
| `20_42_07` | | 13:42 | B | 170 s | 59 m | 26 m | 590 | Return switch |
| `20_46_29` | | 13:46 | B | 400 s | 42 m | 44 m | 2076 | Emergency battery → Land in place |

The two packs were swapped between sessions without marking them, so A and B are not
necessarily 1 and 2.

GPS was good throughout: 18–22 satellites, horizontal accuracy 0.2–0.4 m. Vibration was
acceptable, with no accelerometer clipping except 2 samples in `16_30_56` and the crash
itself. The learned hover thrust was 0.45–0.50.

## The crash (`16_36_24`)

| Time | What happened |
|---|---|
| 160–173 s | Flying away at full forward stick, about 4 m above takeoff height, throttle below centre so it slowly lost height |
| 173.8 s | Crossed the 100 m geofence. PX4 stopped and held for 5 s (`COM_FAIL_ACT_T`) |
| 178.7 s | PX4 started Return (climb to 20 m, fly home) |
| 179.0 s | Return cancelled 0.3 s later, no switch moved: "Pilot took over using sticks" |
| 179–181.5 s | Back in Position mode with throttle at the bottom, i.e. "descend". It sank straight down at about 1 m/s from 4 m |
| 181.5 s | Impact: acceleration peaks of about 290 m/s² (30 g), gyro 1074 °/s, then a fall |
| 183.0 s | Kill switch (SC) engaged by the pilot |
| 188.0 s | Disarmed by the kill switch |

**Cause.** `COM_RC_OVERRIDE` was 1: in auto modes, a stick deflected more than `COM_RC_STICK_OV`
(30 %) from centre counts as the pilot taking over. The Pocket's throttle does not spring back,
and it had been left at the bottom, so PX4 cancelled the geofence Return the moment it began and
obeyed "throttle down". Nothing failed: attitude tracking, GPS, battery and vibration were normal
up to impact. Contributing factors: flying low (about 4 m above takeoff) at 100 m, next to trees,
with the throttle below centre for most of the flight.

**Fix.** `COM_RC_OVERRIDE` = 0, so sticks can't cancel Return, Land or failsafes; take control
with the mode switch instead.

**Damage.** One prop broken, one skid broken, the battery holder came loose (the battery stayed
connected).

## Battery

**PX4 reads current 1.47× low.** Each charger reading is from a full charge, flown, then
recharged:

| Charge | QGC mAh | Charger mAh | Ratio | Meter after flight | Capacity = charger ÷ (1 − meter) |
|---|---|---|---|---|---|
| Pack 1, morning | 2003 | 2970 | 1.48 | 19 % | 3670 mAh |
| Pack A, afternoon | 1894 | 2777 | 1.47 | 23 % | 3610 mAh |
| Pack B, afternoon | 2076 | 3083 | 1.49 | 18 % | 3760 mAh |

The same ratio on three charges and two packs points to the power module's scale factor, not a
battery: `BAT1_A_PER_V` raised from 36.37 to 53.6. True hover current is about 18 A, not the
12.5 A PX4 showed; at 1.69 kg that is about 6 g/W, plausible with ESC losses and frame blockage.
The alternative, a charger over-counting by 47 %, is unlikely, and raising `BAT1_A_PER_V` errs
on the safe side either way.

**Both packs hold about 3700 mAh**, about 70 % of the 5200 on their label: `BAT1_CAPACITY`
set to 3700. Expect about 8 minutes of flight to a 25–30 % reserve.

**The battery % jumped around** (afternoon: 20 % → 14 % in a moment, 13 % → 29 % after
landing): `BAT1_R_INTERNAL` was −1 (auto) and its estimate fell to 0 in flight, so PX4 did not
compensate for voltage sag under load. Every takeoff dropped the estimate by about 20 % within
15 s. The pack's internal resistance, measured from current steps in the logs and corrected for
the current scale, is about 3–4 mΩ per cell: `BAT1_R_INTERNAL` set to 0.004.

## Failsafes

| Log | Trigger | PX4 action | Then |
|---|---|---|---|
| `16_30_56` | Critical battery (141 s) | Hold 5 s, Return | Pilot switched to Position at 146 s, Return switch at 163 s |
| `16_36_24` | Geofence, 100 m (173.8 s) | Hold 5 s, Return | Cancelled by stick override (the crash) |
| `20_29_48` | Critical battery (471 s) | Already returning on the switch | Landed at home |
| `20_46_29` | Critical battery (340 s) | Hold 5 s | Pilot switched to Position at 345 s and climbed |
| `20_46_29` | Geofence, 40 m height (360.7 s) | Hold 4 s, Return (364.6 s) | Emergency battery (366.9 s) → Land in place |

The failsafes triggered as configured each time. Twice the pilot overrode a critical-battery
failsafe and kept flying. In `20_46_29` the emergency level was reached during a 21 A burst
(remaining dipped to 6.8 %, below `BAT_EMERGEN_THR` 7 %), a symptom of the missing sag
compensation, so it landed where it was instead of at home.

## Log download

Logs downloaded through QGC (Analyze Tools → Log Download) were corrupt: each re-download
differed from the previous one in 150–2500 bytes, in clusters of about 90 bytes (one transfer
chunk), and one log could not be read at all. The corrupted copy of the crash log appeared to
end at 121.6 s at 26 m height, which at first suggested a treetop collision. Copies taken
straight from the Pixhawk's microSD card (`log/2026-10-28/`) all read cleanly. **Take logs from
the SD card.**

The logs were read with `pyulog` (in a scratch virtual environment on the laptop).
`tools/plot_flight.py` plots the track, speed and heading of a log given its path.

## Parameters changed

| Parameter | Before | After |
|---|---|---|
| `BAT1_A_PER_V` | 36.37 | 53.6 |
| `BAT1_CAPACITY` | 5200 | 3700 |
| `BAT1_R_INTERNAL` | −1 | 0.004 |
| `COM_RC_OVERRIDE` | 1 | 0 |

## Next

- Replace the broken prop and skid; check the other props, the motor on that corner, its arm and
  mount, and the GPS mast. Secure the battery holder with a strap plus a hook-and-loop pad.
- After the repair, a short low hover to check for new vibration.
- First flight with the new battery parameters, from a full charge: QGC's mAh used should match
  the charger within about 5 %, and the battery % should stay steady under load.
- Keep within 50 m and above tree height when far out, keep the throttle centred in Position
  mode, and let battery failsafes complete.
