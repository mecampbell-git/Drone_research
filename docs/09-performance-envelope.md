# 09 — Performance Envelope & Limits

What this platform can do, what binds first when you push it, and how to explore that safely.

Baseline: X500 V2, 4S 5000 mAh LiPo, Jetson payload, **1.93 kg AUW**.

---

## The envelope

| Parameter | Value | Limited by |
|---|---|---|
| All-up weight | 1.93 kg | — |
| Max thrust | ~3.8 kg | motors + props |
| **Thrust-to-weight** | **~1.97** | — |
| Hover current | ~21 A | — |
| **Peak current** | **~78 A** | **ESCs at 80 A** ⚠ |
| Hover power | ~310 W | — |
| Endurance, hover | 11–13 min | battery energy |
| Endurance, real mission | **8–9 min** | battery + wind + maneuvering |
| Top speed, PX4 default | 12 m/s (27 mph) | `MPC_XY_VEL_MAX` |
| Top speed, achievable | ~22 m/s (49 mph) | tilt limit + drag |
| Best-range speed | 8–12 m/s | aerodynamics |
| Climb, PX4 default | 3 m/s | `MPC_Z_VEL_MAX_UP` |
| Climb, achievable | 8–12 m/s | motor thermal + drag |
| Descent, PX4 default | 1 m/s | **vortex ring state** ⚠ |
| Spare payload (to TWR 1.5) | ~+600 g | control authority |
| Ceiling | 400 ft AGL | **legal**, not physical |
| Range | ~500 m | **legal (VLOS)**, not physical |

---

## What binds first, by axis

| Push on… | What gives first | Failure mode |
|---|---|---|
| **Peak thrust / punch-outs** | **ESCs at 80 A** | ESC overheat or **desync** → a motor stops → crash |
| Sustained speed | Tilt limit, then drag | Simply will not go faster |
| Climb rate | Motor thermal, then vertical drag | Overheating motors |
| **Descent rate** | **Vortex ring state** | Lift collapse, loss of control |
| Payload | TWR toward 1.5, then frame | Sluggish; then arm/connector failure |
| Endurance | Battery energy | Failsafe RTL |
| Range | **Part 107 VLOS** | Legal, long before physical |
| Altitude | **400 ft AGL rule** | Legal, long before air density |

Note how many limits are **legal rather than physical**. Under Part 107 you will hit the rules
well before the airframe complains — which is why "pushing the platform" mostly means exploring
speed, agility and payload, not range or altitude.

---

## ⚠ The ESC ceiling is the real limit

Four 20 A BLHeli_S ESCs give **80 A total** against a theoretical full-throttle demand of ~78 A.
That is roughly 2% margin — the tightest number in the whole design.

Two distinct risks:

**Thermal.** 20 A BLHeli_S ESCs have little thermal mass. Sustained aggressive throttle heats
them fast, and a hot ESC current-limits or shuts down.

**Desync.** BLHeli_S has weaker desync protection than newer firmware. Rapid throttle changes can
cause a motor to lose commutation sync and stop. On a quad, one motor stopping is unrecoverable.
**This is the single most likely way to break this aircraft while exploring the envelope.**

### If you intend to push it, upgrade the ESCs first

Cheapest meaningful headroom available:

| Option | Cost | Gain |
|---|---|---|
| **Reflash to Bluejay or AM32** | free | Better desync protection on the same hardware. Less-trodden with PX4 — verify DShot config carefully. |
| **30–40 A ESCs** (BLHeli_32 / AM32 hardware) | ~$60–80 | Real thermal headroom *and* modern desync protection. **Recommended.** |

Everything else — motors, props, frame — has more margin than the ESCs do. Fix the binding
constraint before spending anywhere else.

---

## Upgrade paths, in order of value per dollar

1. **ESCs, 30–40 A** (~$60–80) — removes the actual ceiling; enables genuine envelope exploration
2. **Li-ion 4S3P pack** (~$150–250) — doubles endurance, costs 22% of climb rate ([03](03-compute-and-power.md))
3. **Bigger props + lower-KV motors** (~$120) — more thrust and better efficiency; requires the ESC upgrade first
4. **6S conversion** (~$250) — same power at lower current, so less ESC stress; needs 6S ESCs and lower-KV motors
5. **650 mm frame** (~$200) — halves disc loading, best endurance gain, but it is a new aircraft

Do them in that order. Each later step assumes the earlier ones.

---

## How to explore the envelope without destroying it

**Take the payload off.** Fly aggressively with a bare airframe and a cheap battery. Learn the
limits, *then* refit the Jetson. Never discover a desync with $500 of compute aboard.

**Change one parameter per flight.** Raise `MPC_XY_VEL_MAX`, or `MPC_TILTMAX_AIR`, or
`MPC_Z_VEL_MAX_UP` — never two. Otherwise you cannot attribute what happened.

**Model it in SITL first.** Parameter changes fly in simulation exactly as they do in the air.
Any limit you intend to raise, raise it in Gazebo first and watch what the aircraft does.

**Touch the motors and ESCs immediately after landing.** Too hot to hold comfortably = back off.
This is the cheapest instrumentation you have.

**Check the logs every flight.** Vibration metrics (`VIBE`), ESC telemetry if available, current
draw, and EKF innovation. Rising vibration as you push is the early warning that matters most for
a vision platform — it degrades IMU, EKF and camera long before anything breaks.

**Stay low and close.** Envelope exploration at 15 m altitude over open grass, with plenty of
horizontal space. Altitude is not your friend when something stops working.

---

## The vision-platform caveat

For a camera-carrying aircraft, **vibration usually limits usable performance well before thrust
does.** Aggressive flight produces jello in the footage, degrades VIO, and injects noise into the
EKF. The aircraft may be perfectly capable of 20 m/s while the perception stack is unusable above
10.

So there are effectively two envelopes: what the airframe can survive, and what the sensors can
work in. Measure both. The second one is the one your missions actually live inside.

---

## Glossary

| Term | Meaning |
|---|---|
| **AGL** | Above Ground Level — altitude measured from the ground directly below, not from sea level. |
| **AUW** | All-Up Weight — total mass of the aircraft ready to fly, including battery and payload. |
| **EKF** | Extended Kalman Filter — the math that fuses IMU, GPS, compass and barometer into a single position and attitude estimate. EKF2 is PX4's version, EKF3 ArduPilot's. |
| **ESC** | Electronic Speed Controller — the board that drives one motor from the flight controller's commands. |
| **IMU** | Inertial Measurement Unit — accelerometers plus gyroscopes. Senses motion and orientation. |
| **jello** | The wobbling, skewed look in video caused by airframe vibration hitting a rolling-shutter camera. |
| **LiPo** | Lithium Polymer — the rechargeable battery chemistry used in drones. "4S" means four cells in series (~14.8 V). |
| **Part 107** | The FAA rule covering commercial/professional drone operation. Requires a remote pilot certificate and VLOS. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RTL** | Return To Launch — automatic flight back to the takeoff point, then land. The standard failsafe. |
| **SITL** | Software In The Loop — running the real flight firmware as a simulation on your PC, no hardware needed. |
| **VIO** | Visual-Inertial Odometry — estimating motion by combining camera images with IMU data. How you navigate without GPS. |
| **VLOS** | Visual Line Of Sight — the operator can see the aircraft unaided. Required under FAA Part 107. |
