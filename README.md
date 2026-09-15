# Drone Research

Research and design notes for an autonomous multirotor using an **NVIDIA Jetson Orin Nano**
as the onboard perception/decision computer.

Status: **research phase — nothing purchased, nothing built.**
Last updated: 2026-09-08

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
| Build path | undecided — see [02-build-paths.md](docs/02-build-paths.md) | open |

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

Buy a **Holybro X500 V2** airframe (~$676, bought decoupled — the bundled kit is sold out), run **PX4 + ROS 2 Jazzy**,
mount the **Orin Nano** as a companion computer on its own DC-DC supply, and start with a single
global-shutter RGB camera. Do every bit of autonomy logic in **PX4 SITL + Gazebo** before it flies.
Add stereo depth once basic offboard control is proven. Total first-milestone spend ≈ **$1,500**,
leaving budget for depth sensing and the inevitable broken props. Rationale in
[02-build-paths.md](docs/02-build-paths.md).

---

## Glossary

| Term | Meaning |
|---|---|
| **ArduPilot** | Open-source flight control firmware; the main alternative to PX4. |
| **DC-DC** | A converter that changes one DC voltage into another (e.g. 16 V battery down to 5 V). |
| **FAA** | Federal Aviation Administration — the US aviation regulator. |
| **GNSS** | Global Navigation Satellite System — the general term for satellite positioning. GPS is the US one. |
| **GPS** | Global Positioning System — satellite positioning. |
| **Isaac ROS** | NVIDIA's set of GPU-accelerated ROS 2 packages for robot perception. |
| **JetPack** | NVIDIA's software bundle for Jetson boards: Linux, CUDA, TensorRT and drivers. |
| **NVMe** | Fast solid-state storage that plugs into an M.2 slot. Far more reliable than a microSD card. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RGB** | Red Green Blue — an ordinary color camera, as opposed to a depth camera. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **SITL** | Software In The Loop — running the real flight firmware as a simulation on your PC, no hardware needed. |
| **SLAM** | Simultaneous Localization And Mapping — building a map while simultaneously working out where you are in it. |
