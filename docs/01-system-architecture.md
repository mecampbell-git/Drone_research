# 01 — System Architecture

## The two-brain split (non-negotiable)

The single most important structural decision, and it is already made for you by physics and
by every serious platform in this space: **the Jetson does not fly the aircraft.**

```
┌─────────────────────────────────────────────────────────────┐
│  JETSON ORIN NANO  —  "what should we do?"                  │
│  Linux, non-realtime, 10–30 Hz                              │
│                                                             │
│  • camera capture + TensorRT inference (detection/tracking) │
│  • depth → local map (nvblox) → path planning               │
│  • visual-inertial odometry (cuVSLAM) when GPS is gone      │
│  • mission logic, geotagged findings, video recording       │
│                                                             │
│  Output: setpoints — "go to this position", "yaw to this    │
│  heading", "hold here". Never motor commands.               │
└────────────────────────┬────────────────────────────────────┘
                         │  UART @ 921600 or Ethernet
                         │  MAVLink  or  DDS (uXRCE / AP_DDS)
┌────────────────────────┴────────────────────────────────────┐
│  FLIGHT CONTROLLER (Pixhawk)  —  "keep us in the air"       │
│  STM32H7, hard realtime, NuttX, 400 Hz–8 kHz                │
│                                                             │
│  • attitude & rate control, motor mixing                    │
│  • EKF fusing IMU / mag / baro / GNSS (/ VIO)               │
│  • RC input, arming, geofence, battery failsafe             │
│  • FAILSAFE: companion goes silent → hold position → RTL    │
└─────────────────────────────────────────────────────────────┘
```

### Why not run everything on the Jetson?

- **Determinism.** Rate control must run at a hard 400 Hz+ with microsecond jitter. Linux
  cannot guarantee that; a page fault or a TensorRT context switch at the wrong moment is a
  crash. The Pixhawk's RTOS can.
- **Boot time.** The FC is flying 2 seconds after power-on. The Jetson takes ~30 s.
- **Failure isolation.** Perception code is the code most likely to have bugs. It must not be
  able to take the aircraft down. If the Jetson panics, hangs, or overheats and throttles to a
  halt, the FC notices the setpoint stream stopped and executes a position hold, then RTL.
- **Certification / community.** Every tuning guide, log analysis tool, and failsafe behavior
  you will lean on assumes a Pixhawk-class FC.

**Design rule:** every autonomous behavior must degrade to a safe state when the Jetson
disappears. Write the failsafe path first, then the feature.

## Control modes and where autonomy plugs in

| Mode | Who commands | Use |
|---|---|---|
| Stabilized / Altitude | Human via RC | Manual flight, tuning, emergency |
| Position (Loiter) | Human via RC, GPS-held | Default safe mode; safety pilot's home base |
| Mission (Auto) | FC executes uploaded waypoints | Survey patterns, "search & inspect" |
| **Offboard** | **Jetson streams setpoints over the link** | **Everything interesting** |

Offboard mode is the seam. PX4 requires a setpoint at ≥2 Hz or it drops out of Offboard —
that requirement *is* the companion-computer watchdog, and it's free.

The safety pilot holds a mode switch on the RC transmitter that yanks the aircraft back to
Position mode instantly, regardless of what the Jetson wants. This is required by FAA rules
(see [07](07-regulatory-and-safety.md)) and is also just how you avoid destroying your drone.

## Data links

| Link | Purpose | Hardware | Criticality |
|---|---|---|---|
| RC control | Safety pilot, mode switching | ExpressLRS **2.4 GHz** or Crossfire | **Highest** — must never fail |
| Companion ↔ FC | Setpoints + state, on-board | UART, 921600 baud (or Ethernet on ARK carriers) | High — loss → position hold |
| Telemetry | GCS, MAVLink, mission upload | SiK **915 MHz** (included in X500 kit) | Medium — flyable without it |
| Mission video + dev | SSH into Jetson, stream **processed** video & metadata | WiFi (later LTE) — **from the Jetson** | None — convenience only |
| Pilot FPV view *(optional)* | Independent human view of where the aircraft points | Analog 5.8 GHz VTX — **own camera, bypasses the Jetson** | None — but valuable as backup |

Note the Jetson's WiFi is a **development convenience, not a control link**. Do not build
anything that requires WiFi to stay airborne.

The RC receiver wires **directly to the flight controller**. The Jetson is not in that path,
by design — no software you write can interfere with the human override.

### Two cameras, two independent paths

The mission video link and the FPV link are **not alternatives** — they serve different
consumers and, conventionally, use different cameras:

```
MISSION CAMERA  (digital — CSI or USB3, global shutter, high-res)
   └─► Jetson ──► detection / depth / VIO ──► setpoints ──► flight controller
                └─► encode ──► WiFi / LTE ──► operator's laptop  (video + bounding boxes)

FPV CAMERA  (analog, tiny, ~$20, ~8 g)   [optional, fully independent]
   └─► analog VTX 5.8 GHz ──────────────────────────► pilot's goggles or screen
```

**The FPV path never touches the Jetson**, and that is the entire point. The VTX is only
"independent eyes" if it has its own camera and its own transmitter. An analog FPV camera
outputs composite video straight into the VTX at ~15 ms latency with no computer in the loop.

**Do not** route the pilot's view through the Jetson (HDMI out → composite converter). It works,
but it makes the pilot go blind exactly when the companion computer fails — the moment they most
need to see — and adds 100 ms+ of latency to save 8 grams.

**Do you need FPV at all?** Probably not at first. Part 107 requires visual line of sight, so you
are flying by looking at the aircraft; many PX4 research platforms carry no FPV system, and the
X500 kit does not include one. **Skip it initially, add it around M6** — once the aircraft starts
moving on its own, an independent view of where it is pointing is worth ~$50 (camera + VTX +
receiver).

Only the **mission camera** is required. Everything in [04](04-perception-sensors.md) concerns
that one.

### Frequency planning — do not put two radios in the same band

Running ExpressLRS at 915 MHz alongside the SiK telemetry radio at 915 MHz puts two
transmitters centimetres apart in the same band. They desense each other and you lose real
range on both. Pick a combination that separates them:

| Option | Verdict |
|---|---|
| **ELRS 2.4 GHz + SiK 915 MHz** | ⭐ Clean separation, both unlicensed in the US. 2.4 GHz ELRS has far more range than VLOS needs. |
| ELRS 915 MHz + SiK 433 MHz | Separated, but **433 MHz is amateur band in the US** — needs a ham licence at these power levels. The X500 kit offers a 433 variant; it is aimed at other regions. |
| Drop SiK, run MAVLink over the Jetson's WiFi/LTE | Couples telemetry to the companion computer, which this architecture deliberately avoids. |

**Order the 915 MHz SiK variant of the X500 kit and pair it with 2.4 GHz ExpressLRS.**

One consequence to plan around: **USB 3.0 radiates broadband noise right in the 2.4 GHz band**,
which degrades both ELRS 2.4 and the Jetson's WiFi. Mitigations, in order of preference:
prefer a **CSI camera over USB3** (already recommended in [04](04-perception-sensors.md) for
latency reasons — this is a second reason), use shielded cable with ferrites, and keep RX
antennas physically distant from the Jetson, its NVMe, and any USB3 run.

## Coordinate frames — the thing that will bite you

Three frames, and mixing them up is the #1 source of "why did it fly into a tree":

- **Camera / optical** — X right, Y down, Z forward (OpenCV convention)
- **Body / FRD** — X forward, Y right, Z down (PX4, MAVLink)
- **World / NED** — North, East, Down (PX4 local frame, origin at arming point)

ROS 2 conventions (REP-103) use FLU/ENU instead, so the ROS↔PX4 boundary needs an explicit
transform. `px4_ros_com` ships helpers for this. Get the static camera→body extrinsics
measured properly and into a URDF/TF tree early; every downstream perception result depends on it.

---

## Glossary

| Term | Meaning |
|---|---|
| **ARK** | ARK Electronics — maker of integrated flight-controller + Jetson carrier boards. |
| **CSI** | Camera Serial Interface (MIPI CSI) — a direct ribbon-cable camera connection. Lower latency and CPU cost than USB. |
| **cuVSLAM** | NVIDIA's GPU-accelerated visual SLAM implementation, shipped as part of Isaac ROS. |
| **DDS** | Data Distribution Service — the publish/subscribe messaging standard underneath ROS 2. |
| **EKF** | Extended Kalman Filter — the math that fuses IMU, GPS, compass and barometer into a single position and attitude estimate. EKF2 is PX4's version, EKF3 ArduPilot's. |
| **ExpressLRS** | ELRS — open-source long-range radio control link for the manual pilot. |
| **FAA** | Federal Aviation Administration — the US aviation regulator. |
| **failsafe** | Pre-programmed automatic behavior when something goes wrong (link lost, battery low, companion computer silent). |
| **FC** | Flight Controller — the real-time board (a Pixhawk here) that actually stabilizes and flies the aircraft. |
| **FPV** | First Person View — flying by watching a live camera feed from the aircraft. |
| **GCS** | Ground Control Station — the laptop or tablet software (e.g. QGroundControl) that monitors and commands the drone. |
| **geofence** | A virtual boundary. Crossing it triggers an automatic response such as RTL. |
| **GNSS** | Global Navigation Satellite System — the general term for satellite positioning. GPS is the US one. |
| **GPS** | Global Positioning System — satellite positioning. |
| **IMU** | Inertial Measurement Unit — accelerometers plus gyroscopes. Senses motion and orientation. |
| **LTE** | Mobile phone data network (4G). 5G is its successor. |
| **MAVLink** | The standard lightweight messaging protocol between drones, ground stations and companion computers. |
| **NED** | North-East-Down — a coordinate frame convention. Related: FRD (Forward-Right-Down), FLU (Forward-Left-Up), ENU (East-North-Up). |
| **NuttX** | The real-time operating system running on the Pixhawk flight controller. |
| **nvblox** | NVIDIA's GPU library that builds a live 3D map from depth images. |
| **NVMe** | Fast solid-state storage that plugs into an M.2 slot. Far more reliable than a microSD card. |
| **Offboard** | PX4's flight mode in which the companion computer streams position setpoints. Where all autonomy happens. |
| **Part 107** | The FAA rule covering commercial/professional drone operation. Requires a remote pilot certificate and VLOS. |
| **Pixhawk** | The open hardware standard for drone flight controllers. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RC** | Radio Control — the manual pilot's link to the aircraft. |
| **REP-103** | The ROS convention document defining standard units and coordinate frames. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **RTL** | Return To Launch — automatic flight back to the takeoff point, then land. The standard failsafe. |
| **RTOS** | Real-Time Operating System — one that guarantees timing. NuttX is the RTOS on the Pixhawk. |
| **setpoint** | A commanded target — "go to this position", "hold this heading". What the Jetson sends the flight controller. |
| **SiK** | The firmware (and the radios running it) used for standard 915/433 MHz MAVLink telemetry. |
| **SSH** | Secure Shell — remote terminal login over the network. |
| **STM32H7** | The microcontroller chip inside a Pixhawk flight controller. |
| **TensorRT** | NVIDIA's library that optimizes a trained neural network to run fast on a specific GPU. |
| **TF** | The ROS coordinate-frame transform system (`tf2`). Tracks where every part and sensor sits relative to everything else. |
| **UART** | A simple two-wire serial connection between boards. How the Jetson talks to the flight controller. |
| **URDF** | Unified Robot Description Format — an XML file describing the robot's parts and exactly where each sensor sits. |
| **USB3** | USB 3.0 — fast USB, and a notorious source of 2.4 GHz radio noise. |
| **VIO** | Visual-Inertial Odometry — estimating motion by combining camera images with IMU data. How you navigate without GPS. |
| **VLOS** | Visual Line Of Sight — the operator can see the aircraft unaided. Required under FAA Part 107. |
| **VTX** | Video Transmitter — the FPV video downlink to the pilot. |
