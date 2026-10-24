# 06 — Autonomy Roadmap

Nine milestones, ordered by dependency. Each has an explicit **done** condition — a thing that
either works or does not, so you are never guessing whether to move on.

The four stated goals map onto milestones 5, 6, 7, and 9. Milestones 0–4 are the price of
admission for all of them.

---

### M0 — SITL loop closed *(cost: $0, do this now)*

PX4 SITL + Gazebo on the desktop. A ROS 2 Jazzy node connects over uXRCE-DDS, arms, takes off,
flies a 10 m square, lands.

**Done:** the square flies, repeatably, from a single `ros2 launch`.

#### What "the square" means, and why it is the right first test

An **offboard node** is the ROS 2 program you write that streams position setpoints to PX4 while
it is in Offboard mode. "Node" is ROS's word for one program in the graph. This is *your* code —
the thing that later grows into mission logic.

**Flying the square** is the hello-world of autonomous flight: arm → take off to ~5 m → corner 1
→ 2 → 3 → 4 → return → land, on a 10 m square.

It is the smallest test that exercises the whole chain *and* makes its failures visible:

| Exercises | Failure looks like |
|---|---|
| **Mode entry** — PX4 needs setpoints streaming *before* it will accept Offboard | Never enters Offboard at all (classic first-timer trap) |
| **Setpoint rate** — must stay ≥2 Hz or PX4 drops out | Falls out of Offboard partway round |
| **Coordinate frames** — NED vs ENU, X vs Y | Square comes out rotated, mirrored, or traversed in the wrong order |
| **Arrival logic** — when have you reached a corner? | Hovers forever (tolerance too tight) or rounds off corners (too loose) |
| **Sequencing** — the minimal mission state machine | Skips or repeats legs |

⭐ **Corners are what make frame errors legible.** A circle or a single point-to-point hop would
hide exactly the bug most likely to be present. You diagnose this test by *looking at a plot*.

**10 m** is sized so GPS noise (±1–2 m) does not swamp the shape, while staying small enough for a
modest test site well inside VLOS.

The square appears twice on purpose: **M0 flies it in simulation, M4 flies the same node on the
real aircraft.** Per the fidelity ladder in [05](05-software-stack.md) that should be a
byte-identical binary — so when the square that worked in Gazebo comes out wrong outdoors, the
difference is hardware, not logic.

---

### M1 — Airframe flies manually

Assemble the X500 V2. Bind RC. Calibrate sensors. Tune. Fly ~10 manual battery packs until
takeoff, hover, and landing are muscle memory. Practice failsafe: kill the RC link deliberately
at altitude and confirm RTL.

Step by step: [10 — Hardware Bring-up](10-hardware-bringup.md).

**Done:** you are bored flying it, and RTL has been verified in the air.

*Do not skip this.* When the autonomy misbehaves, your ability to take over instantly is the
only thing between the project and a pile of carbon fiber.

---

### M2 — Jetson on the bench, talking to the FC

**Props removed. Physically in another room.** JetPack 7.2 is already installed; boot from NVMe, install ROS 2 Jazzy +
Isaac ROS. Wire the UART. Confirm the Jetson reads live telemetry — attitude, position, battery
— from the real Pixhawk.

**Done:** `ros2 topic echo` shows live vehicle attitude changing as you tilt the airframe by hand.

---

### M2.5 — Jetson runs the real code against a simulated aircraft

Still props-off. Run your M0 nodes **on the Jetson itself**, with PX4 SITL + Gazebo on the
desktop, connected over the network. Same code, real ARM64 build, real TensorRT engine, real
8 GB memory limit — simulated flight.

**Done:** the M0 square flies from Jetson-hosted nodes, and you have measured the real inference
rate and memory headroom on target hardware.

*This is rung 2 of the fidelity ladder in [05](05-software-stack.md).* It is where you find out
your detector runs at 12 fps rather than 60, or that a dependency has no ARM64 wheel — while
crashes are still free. Skipping it means discovering those things in M5, disguised as
mysterious autonomy bugs.

---

### M3 — Jetson flies as ballast

Mount the Jetson, DC-DC, and camera. Fly manually with everything powered and recording, but
commanding nothing. Watch: current draw, `tegrastats` temperature, vibration levels in the PX4
log, and video for jello.

**Done:** a full pack flown with no thermal throttling, no brownouts, clean IMU vibe metrics,
and known real flight time.

*This is the milestone people skip and regret.* Power and thermal problems found here are cheap;
found in M5 they look like mysterious autonomy bugs.

---

### M3.5 — Dress rehearsal: HITL

**Props off.** Real Pixhawk in HITL mode fed simulated sensors by Gazebo Classic on the desktop;
real Jetson wired to **TELEM2 at 921600** running the M0 node; your real transmitter bound and in
your hands.

```
Desktop        Gazebo Classic — physics + simulated sensors
   ↕ USB
Pixhawk 6C     real PX4 firmware, HITL mode, props off
   ↕ TELEM2 UART @ 921600            ← the real link, under real load
Jetson         your ROS 2 nodes, real ARM build
   ↕ RC receiver ← your transmitter
```

**Done:** the M0 square flies in HITL, commanded by the Jetson over the real UART — and flipping
the RC mode switch takes control back instantly, verified several times.

**What this catches that M0–M3 cannot:**

- **The real companion↔FC serial link under closed-loop load.** M2 proved the Jetson can *read*
  telemetry; this proves it can *command* through a 921600-baud UART with real latency and
  dropouts. In SITL that path is localhost UDP — effectively perfect, and therefore untested.
- **The manual override, rehearsed.** You practise the emergency that M4 depends on, with the real
  radio and real firmware, against an aircraft that cannot be damaged.
- **Real firmware on real silicon** — ARM build, STM32H7 timing, actual parameter storage and boot
  behaviour.

*Rung 3 of the fidelity ladder in [05](05-software-stack.md), and the last free rehearsal before
M4 costs money.* Setup details and caveats — `SYS_HITL`, `pwm_out_sim`, Gazebo **Classic** rather
than modern Gazebo — are in that document.

**Can be pulled earlier** (before M3) if the UART proves troublesome at M2 and you want to
characterise it before flying anything.

---

### M4 — Offboard control on the real aircraft

The M0 node, unchanged, now commands the real drone. Outdoors, open field, safety pilot with a
finger on the mode switch, generous geofence.

**Done:** the same 10 m square flies for real, and flipping the mode switch returns manual
control instantly, every time.

**This is the project's keystone.** Everything after it is perception feeding a control interface
you have already proven.

---

### M5 — Detection in flight *(goal: search & inspect)*

TensorRT detector on the live camera stream, running in flight. Nothing commands the aircraft yet
— fly a manual or waypoint pattern and log detections with timestamp, GPS position, and aircraft
attitude.

**Done:** a flight produces a geotagged list of detections you can plot on a map, plus the
end-to-end latency number.

---

### M6 — Closed-loop visual servoing *(goal: detect / track / follow)*

Detections become setpoints: yaw to keep the target centered, then hold standoff distance.

**Done:** the aircraft holds a moving target in frame autonomously, and loses it gracefully —
target gone for N seconds → hover, not drift.

**The hard part is not the tracking, it is the failure handling.** Write the "what if detection
returns nothing / two targets / a false positive" logic in SITL first.

---

### M7 — Depth and local mapping *(goal: obstacle avoidance)*

Add the D435i. Run `nvblox` to build a live ESDF cost map. First passively — visualize the map
while flying manually — then feed it to a local planner that adjusts setpoints.

**Done:** the aircraft flies a commanded path that would intersect an obstacle and deviates
around it, with the safety pilot never touching the sticks.

Expect this to take longer than M0–M6 combined. Test at walking pace, at 2 m altitude, over grass,
with an obstacle you do not mind hitting.

---

### M8 — GPS-denied flight *(conditional on your environment decision)*

`isaac_ros_visual_slam` publishes pose; PX4 EKF2 fuses it as an external vision estimate.
Validate outdoors with GNSS available (compare VIO pose against GPS truth) **before** trusting it
indoors.

**Done:** stable position hold with GPS deliberately disabled, outdoors, then indoors.

---

### M9 — Mapping / reconstruction *(goal: 3D mapping)*

Either offline photogrammetry from logged geotagged imagery, or live `nvblox` mesh export.

**Done:** a georeferenced 3D model of a real site from one flight.

---

## Cross-cutting rules

1. **Every milestone gets a SITL version first.** No exceptions, including M7.
2. **Every autonomous behavior needs a defined degraded state** — companion dies, detection dies,
   depth dies, link dies. Write that path before the happy path.
3. **Log everything, every flight.** PX4 `.ulg` + `ros2 bag` + raw video. Storage is cheap;
   an unreproducible in-flight anomaly is not.
4. **One variable per flight.** Changing the model and the controller gains in the same test
   means you learn nothing from the result.
5. **Safety pilot, always.** Also an FAA requirement — see [07](07-regulatory-and-safety.md).

---

## Glossary

| Term | Meaning |
|---|---|
| **DC-DC** | A converter that changes one DC voltage into another (e.g. 16 V battery down to 5 V). |
| **DDS** | Data Distribution Service — the publish/subscribe messaging standard underneath ROS 2. |
| **ESDF** | Euclidean Signed Distance Field — a 3D grid storing the distance to the nearest obstacle. What a path planner reads to avoid things. |
| **FAA** | Federal Aviation Administration — the US aviation regulator. |
| **failsafe** | Pre-programmed automatic behavior when something goes wrong (link lost, battery low, companion computer silent). |
| **FC** | Flight Controller — the real-time board (a Pixhawk here) that actually stabilizes and flies the aircraft. |
| **geofence** | A virtual boundary. Crossing it triggers an automatic response such as RTL. |
| **GNSS** | Global Navigation Satellite System — the general term for satellite positioning. GPS is the US one. |
| **GPS** | Global Positioning System — satellite positioning. |
| **HITL** | Hardware In The Loop — real flight controller and Jetson wired to a simulated world. |
| **IMU** | Inertial Measurement Unit — accelerometers plus gyroscopes. Senses motion and orientation. |
| **Isaac ROS** | NVIDIA's set of GPU-accelerated ROS 2 packages for robot perception. |
| **jello** | The wobbling, skewed look in video caused by airframe vibration hitting a rolling-shutter camera. |
| **JetPack** | NVIDIA's software bundle for Jetson boards: Linux, CUDA, TensorRT and drivers. |
| **NED** | North-East-Down — a coordinate frame convention. Related: FRD (Forward-Right-Down), FLU (Forward-Left-Up), ENU (East-North-Up). |
| **nvblox** | NVIDIA's GPU library that builds a live 3D map from depth images. |
| **NVMe** | Fast solid-state storage that plugs into an M.2 slot. Far more reliable than a microSD card. |
| **Offboard** | PX4's flight mode in which the companion computer streams position setpoints. Where all autonomy happens. |
| **Pixhawk** | The open hardware standard for drone flight controllers. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RC** | Radio Control — the manual pilot's link to the aircraft. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **RTL** | Return To Launch — automatic flight back to the takeoff point, then land. The standard failsafe. |
| **SITL** | Software In The Loop — running the real flight firmware as a simulation on your PC, no hardware needed. |
| **STM32H7** | The microcontroller chip inside a Pixhawk flight controller. |
| **TensorRT** | NVIDIA's library that optimizes a trained neural network to run fast on a specific GPU. |
| **UART** | A simple two-wire serial connection between boards. How the Jetson talks to the flight controller. |
| **uXRCE-DDS** | The bridge that lets PX4 publish and subscribe to ROS 2 topics directly, without a MAVLink translation layer. |
| **VIO** | Visual-Inertial Odometry — estimating motion by combining camera images with IMU data. How you navigate without GPS. |
| **VLOS** | Visual Line Of Sight — the operator can see the aircraft unaided. Required under FAA Part 107. |
