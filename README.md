# Drone Research

Research and design notes for an autonomous multirotor using an **NVIDIA Jetson Orin Nano**
as the onboard perception/decision computer.

Status: **parts ordered; M0 (SITL) in progress.** See [Hardware](#hardware) and [Code and simulation](#code-and-simulation).
Last updated: 2026-10-06

---

## Goal

A 7–10" class quadcopter that can:

1. **Detect / track / follow** — vision model finds a target, aircraft keeps it framed or follows it
2. **Search & inspect** — fly a pattern, run detection on the live stream, geotag and report findings
3. **Avoid obstacles / plan paths** — depth perception, local mapping, navigate clutter
4. **Map / reconstruct** — photogrammetry or live SLAM of a site

Specific mission is deliberately undefined. The design target is a **capable, reusable
autonomy testbed** rather than a single-purpose aircraft.

## Fixed constraints

| Constraint | Value | Source |
|---|---|---|
| Compute | Jetson Orin Nano, **JetPack 7.2**, 128 GB NVMe, 183 g (already owned) | given |
| Size class | 7–10" props, ~500mm class frame | chosen |
| Budget | $1,500–3,000 all-in | chosen |
| Environment | Outdoor/GNSS first, **must not foreclose** indoor/GPS-denied | chosen |
| Build path | **A: X500 V2 kit + Jetson retrofit**, see [02-build-paths.md](docs/02-build-paths.md) | chosen |

## Read in this order

| Doc | What it answers |
|---|---|
| [01-system-architecture.md](docs/01-system-architecture.md) | How the pieces fit together, and why the Jetson is *not* the flight controller |
| [02-build-paths.md](docs/02-build-paths.md) | Kit vs. integrated vs. scratch vs. sim-first — with real BOMs and prices |
| [03-compute-and-power.md](docs/03-compute-and-power.md) | Getting the Orin Nano airborne: carrier, power, thermal, vibration, weight |
| [04-perception-sensors.md](docs/04-perception-sensors.md) | Cameras and depth sensing — the RealSense situation and what to buy instead |
| [05-software-stack.md](docs/05-software-stack.md) | JetPack, ROS 2, Isaac ROS, PX4/ArduPilot, simulation |
| [06-autonomy-roadmap.md](docs/06-autonomy-roadmap.md) | Staged milestones from SITL to obstacle avoidance |
| [07-regulatory-and-safety.md](docs/07-regulatory-and-safety.md) | FAA rules that constrain the design, and the safety architecture |
| [08-mission-data-flow.md](docs/08-mission-data-flow.md) | Search → operator selects → investigate: how information actually flows, and the video downlink |
| [09-performance-envelope.md](docs/09-performance-envelope.md) | What the platform can do, what binds first when you push it, and how to explore that safely |
| [open-questions.md](docs/open-questions.md) | Decisions still to make, and what would settle them |

## The one-paragraph summary

Build on a **Holybro X500 V2** airframe (bought decoupled, since the bundled kit is sold out), run **PX4 + ROS 2 Jazzy**,
mount the **Orin Nano** as a companion computer on its own DC-DC supply, and start with a single
global-shutter RGB camera (an e-con See3CAM_24CUG over USB3). Do every bit of autonomy logic in **PX4 SITL + Gazebo** before it flies.
Add stereo depth once basic offboard control is proven. Total first-milestone spend ≈ **$1,500**,
leaving budget for depth sensing and the inevitable broken props. Rationale in
[02-build-paths.md](docs/02-build-paths.md).

## Hardware

### Ordered

| Item | Price | Notes |
|---|---|---|
| Holybro X500 V2 ARF Kit | $370.49 | Frame, motors, ESCs, props, PDB |
| Holybro Pixhawk 6C, plastic case | $179.99 | |
| Holybro PM02 V3 12S power module | $24.99 | Analog, the correct pairing for the 6C (not the PM02D) |
| Holybro M10 GPS, 10-pin, IST8310 | $52.13 | |
| Holybro SiK Telemetry Radio V3, 915 MHz, pair | $71.08 | |
| RadioMaster Pocket, ELRS 2.4 GHz | $84.49 | Safety pilot radio |
| RadioMaster RP3 V2 ELRS receiver | $22.22 | Solder pads, no plug: needs a JST-GH lead to TELEM3 |
| Dronetag Beacon | $79.00 | Standalone Remote ID broadcast module with its own GPS and battery |
| e-con See3CAM_24CUG, no enclosure | — | AR0234 global shutter, USB3 UVC, onboard ISP, 104.6° HFOV lens |
| **Subtotal** | **$884.39** | Excluding the camera, tax and shipping |

Already owned: Jetson Orin Nano Super Dev Kit (JetPack 7.2, 128 GB NVMe, 183 g measured).

### Still needed before first flight

| Item | ~Price | Notes |
|---|---|---|
| 4S 6500 mAh LiPo ×3 (XT60) + charger | ~$220+ | OVONIC chosen: ~14.6 min hover vs ~12.2 min for a 5200. Charger must do ≥6.5 A per pack. |
| 19 V / 5 A DC-DC buck-boost | ~$25 | Jetson supply, see [03](docs/03-compute-and-power.md) |
| JST-GH 6-pin to Dupont cable | ~$10 | Pixhawk TELEM2 to the Jetson UART |
| 2× 18650 cells | ~$15 | For the Pocket radio |
| Mounting kit | ~$60–80 | Standoffs, Jetson damping grommets, 10 mm tube clamps, VHB, zip ties, heat-shrink, antenna tubes |
| Aluminum or G10 sheet | ~$15 | Downward sensor plate for the camera (and later the rangefinder) |
| LiPo safety bag + voltage checker | ~$20 | |
| Spare props | ~$15 | |

Roughly **$400–450** more, plus a soldering iron (~$30–60) if you don't have one.

## Code and simulation

| Path | Contents |
|---|---|
| `docs/` | The design notes listed above |
| `ws/` | colcon workspace: `px4_offboard` (offboard, teleop and camera nodes), `px4_bringup` (SITL launch), `px4_msgs` (submodule, `release/1.17`) |
| `sim/` | Gazebo models and PX4 airframes for the simulated X500 with camera |

Stack: ROS 2 Jazzy, Ubuntu 24.04, Gazebo Harmonic, PX4 v1.17.0 in `~/PX4-Autopilot`.

```bash
git clone --recursive <this repo>              # or: git submodule update --init
sim/link_into_px4.sh                           # link models and airframes into ~/PX4-Autopilot
cd ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch px4_bringup minimal.launch.py world:=baylands
```

Development details and gotchas are in [CLAUDE.md](CLAUDE.md).

---

## Glossary

| Term | Meaning |
|---|---|
| **ArduPilot** | Open-source flight control firmware; the main alternative to PX4. |
| **DC-DC** | A converter that changes one DC voltage into another (e.g. 16 V battery down to 5 V). |
| **ELRS** | ExpressLRS — open-source long-range radio control link for the manual pilot. |
| **FAA** | Federal Aviation Administration — the US aviation regulator. |
| **Gazebo** | The robot simulator PX4 SITL runs in. |
| **GNSS** | Global Navigation Satellite System — the general term for satellite positioning. GPS is the US one. |
| **GPS** | Global Positioning System — satellite positioning. |
| **Isaac ROS** | NVIDIA's set of GPU-accelerated ROS 2 packages for robot perception. |
| **ISP** | Image Signal Processor — turns raw sensor data into a usable color image (white balance, exposure, demosaicing). |
| **JetPack** | NVIDIA's software bundle for Jetson boards: Linux, CUDA, TensorRT and drivers. |
| **NVMe** | Fast solid-state storage that plugs into an M.2 slot. Far more reliable than a microSD card. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **Remote ID** | FAA requirement (14 CFR Part 89) that a drone broadcast its identity and location while flying. |
| **RGB** | Red Green Blue — an ordinary color camera, as opposed to a depth camera. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **SITL** | Software In The Loop — running the real flight firmware as a simulation on your PC, no hardware needed. |
| **SLAM** | Simultaneous Localization And Mapping — building a map while simultaneously working out where you are in it. |
| **UVC** | USB Video Class — the standard webcam protocol. Works with Linux's built-in driver. |
