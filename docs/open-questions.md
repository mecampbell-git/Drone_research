# Open Questions

Decisions not yet made, with what would settle each. Update as they close.

---

### Q1 — Indoor / GPS-denied: is it actually a requirement?
**Status:** open (answered "not sure yet")
**Why it matters:** drives VIO, camera choice, payload margin, and roughly 2 months of work.
**Deferred safely by:** buying a global-shutter camera with a synchronized IMU and leaving
payload/IO margin ([04](04-perception-sensors.md)). No hardware is foreclosed.
**Settles when:** you name a first concrete mission. Revisit at M5.

---

### Q2 — What is the actual first mission?
**Status:** ✅ answered — **simple area survey**, with the platform kept general.

The stated intent is a survey mission first, but with the aircraft treated as a **general-purpose
testbed whose limits are worth knowing**, since objectives may change and the envelope is
interesting in its own right.

**Design consequence:** do not over-specialise for survey. Keep payload margin, keep the sensor
suite open, and document the performance envelope rather than just the survey operating point —
see [09-performance-envelope.md](09-performance-envelope.md).

**Still worth pinning down before M5:** a concrete *detection target* (what is the model looking
for?), since that drives camera FOV, altitude and training data even though the flight profile is
settled.

---

### Q3 — Development environment
**Status:** ✅ answered — **the RTX 5070 Ti machine is dual-booted with Ubuntu 24.04 and used for
everything**: PX4 SITL, Gazebo, ROS 2 Jazzy, HITL, fine-tuning, and Isaac Sim at M7.

| Machine | Role | OS |
|---|---|---|
| RTX 5070 Ti (16 GB Blackwell) | Everything | Ubuntu 24.04, dual-boot with Windows |
| Laptop | Field ops | Omarchy, unchanged |
| i7-8700, 16 GB | Spare | — |

**Monitor on motherboard HDMI is confirmed working** (desktop uses no VRAM), which removes most
Blackwell driver risk. Before installing: **save the BitLocker recovery key** and **disable Fast
Startup**. Full checklist in [05-software-stack.md](05-software-stack.md).

---

### Q4 — Flight controller: Pixhawk 6C or 6X?
**Status:** leaning 6C
**Trade:** $579 vs. $669 for the kit. 6X adds triple redundant IMUs and vibration isolation.
**Argument for 6C:** the redundancy matters for commercial reliability, not for a testbed where
you are the failure mode. Spend the $90 on a third battery.
**Argument for 6X:** better IMU isolation genuinely helps VIO, if Q1 resolves to "yes".
**Settles when:** Q1 settles. If ordering before then, take the 6C.

---

### Q5 — When to move to an integrated carrier (ARK)?
**Status:** deferred by design
**Trigger:** when UART flakiness, payload margin, or wiring mess becomes the thing slowing you
down — likely around M7. Revisit then, with real data instead of speculation.

---

### Q6 — Endurance target?
**Status:** answered for now — **starting on LiPo** (~12 min hover / ~8 min mission), which is the
right choice while flights are short test sorties. Li-ion 4S3P (~$150–250) roughly doubles it to
~24 min at the cost of 22% climb rate; revisit when survey endurance actually binds. See
[03](03-compute-and-power.md).

---

### Q9 — Upgrade the ESCs before pushing the envelope?
**Status:** open, but recommended
**Why it matters:** the stock 4× 20 A BLHeli_S ESCs are the platform's binding constraint (~2%
margin at full throttle) and BLHeli_S desync is the most likely way to break the aircraft while
exploring limits. 30–40 A ESCs are ~$60–80.
**Settles when:** you decide whether envelope exploration is a near-term goal. Not needed for
survey work. See [09](09-performance-envelope.md).

---

### Q7 — Repo not yet under version control
**Status:** actionable now
`git init` before any code lands. The ROS 2 workspace, PX4 params, URDF/extrinsics, and
checklists all belong in history.

---

### Q8 — Will you operate outside cellular coverage?
**Status:** open
**Why it matters:** decides whether the operator link is stock WiFi, an RFD900x upgrade, or a
$1,500+ MANET radio. See "Operating without cellular coverage" in
[08](08-mission-data-flow.md).
**Cheap answer for now:** under Part 107 VLOS the operator is co-located, so a local link is
sufficient and no infrastructure is needed. Revisit only if BVLOS or remote-team review becomes
a real requirement.

---

### Q10 — Precise velocity/acceleration control from Python custom modes
**Status:** deferred (2026-10-08) — Goto at 5 m/s is good enough for now; moving on to vision.

The Python waypoint mission ([`waypoint_mission.py`](../ws/src/px4_offboard/px4_offboard/waypoint_mission.py))
flies with PX4's **Goto** setpoint. That gives PX4's smoothing for free but limits control:
- speed is capped by PX4's `MPC_XY_CRUISE` (default 5 m/s) and acceleration by `MPC_ACC_HOR`,
  scaled by `max_speed / MPC_XY_CRUISE`; neither parameter can be changed from a ROS 2 node;
- Goto slows proportionally as it nears its target (~0.5 × distance, `MPC_XY_TRAJ_P`), which is
  why fly-through waypoints need the "carrot" workaround.

**The fix:** bind the library's **Trajectory** setpoint type (`px4_ros2::TrajectorySetpointType`,
`control/setpoint_types/experimental/trajectory.hpp`): position, velocity, acceleration and yaw,
each optional, the same as offboard's `TrajectorySetpoint`. It exists in the 1.17 C++ library but
has no Python binding; adding one is the same small job as the bindings already added to the
vendored [`px4_ros2_py`](../ws/src/px4_ros2_py/README.md). Then a mode can run its own speed
profile with velocity feed-forward, fly through corners without the carrot, and exceed
`MPC_XY_CRUISE` (up to `MPC_XY_VEL_MAX`).
**Settles when:** a task needs precise velocity control or speeds above 5 m/s — at the latest
M6 (visual servoing, which streams velocity commands from the tracker) or M7 (avoidance).

---

## Glossary

| Term | Meaning |
|---|---|
| **ARK** | ARK Electronics — maker of integrated flight-controller + Jetson carrier boards. |
| **BVLOS** | Beyond Visual Line Of Sight — flying where the operator cannot see the aircraft. Needs special FAA authorization. |
| **FOV** | Field Of View — how wide an angle the camera sees. |
| **GPS** | Global Positioning System — satellite positioning. |
| **HITL** | Hardware In The Loop — real flight controller and Jetson wired to a simulated world. |
| **IMU** | Inertial Measurement Unit — accelerometers plus gyroscopes. Senses motion and orientation. |
| **LiPo** | Lithium Polymer — the rechargeable battery chemistry used in drones. "4S" means four cells in series (~14.8 V). |
| **MANET** | Mobile Ad-hoc NETwork — radios that mesh together and relay for each other. |
| **Part 107** | The FAA rule covering commercial/professional drone operation. Requires a remote pilot certificate and VLOS. |
| **Pixhawk** | The open hardware standard for drone flight controllers. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RFD900x** | A long-range (40 km+) telemetry radio; a drop-in upgrade from the SiK radio. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **RTX** | NVIDIA's desktop GPU line. Isaac Sim needs one. |
| **SITL** | Software In The Loop — running the real flight firmware as a simulation on your PC, no hardware needed. |
| **UART** | A simple two-wire serial connection between boards. How the Jetson talks to the flight controller. |
| **URDF** | Unified Robot Description Format — an XML file describing the robot's parts and exactly where each sensor sits. |
| **VIO** | Visual-Inertial Odometry — estimating motion by combining camera images with IMU data. How you navigate without GPS. |
| **VLOS** | Visual Line Of Sight — the operator can see the aircraft unaided. Required under FAA Part 107. |
