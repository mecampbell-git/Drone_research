# 04 — Perception & Sensors

## Start with one camera

Resist buying a depth camera on day one. Goals 1 and 2 (detect/track/follow, search & inspect)
need **only RGB**. Goals 3 and 4 (avoidance, mapping) need depth. Build in that order — the
first two goals will teach you the integration, and by the time you need depth you will know
exactly which depth camera you want.

## What matters in a drone camera

| Property | Why |
|---|---|
| **Global shutter** | Non-negotiable. Rolling shutter + airframe vibration = skewed, jellied frames that wreck both detection and VIO. This single spec matters more than resolution. |
| **Field of view** | Wide (>90° H) for navigation and avoidance; narrow for inspection detail. You cannot have both in one fixed lens — pick per mission or carry two. |
| **Interface** | CSI/MIPI beats USB3 on a Jetson: lower latency, lower CPU load, no USB bandwidth contention. The Orin Nano dev kit carrier exposes 2× MIPI CSI connectors. USB3 is more plug-and-play; CSI needs a working driver for your exact sensor. |
| **Synchronization** | For VIO, camera exposure must be time-aligned with IMU samples. Cameras with a hardware sync/trigger and an onboard IMU (D435i, ZED) make this dramatically easier. |
| **Weight & power** | Every gram and watt, see [03](03-compute-and-power.md). |

## Depth options (for when you get there)

**RealSense is healthy again** — worth stating because the internet is full of stale "RealSense is
dead" posts. Intel spun it out; **RealSense, Inc. completed the spinout on 2025-07-11** with a
$50M Series A (Intel Capital, MediaTek) and an announced strategic collaboration with NVIDIA.
It is shipping new hardware in 2026 (D585 Pro, a GMSL-enabled lineup). Supply risk is much lower
than it looked in 2024.

| Camera | Type | Notes for this build |
|---|---|---|
| **RealSense D435i** | Active IR stereo + IMU | The default. Global shutter, wide FOV, onboard IMU, best-documented ROS 2 support, huge drone community. IR projector is useless in sunlight but the **passive stereo still works outdoors** — the projector only helps on indoor textureless walls. |
| **RealSense D455** | Active IR stereo + IMU | Longer baseline → better depth at range. Heavier and larger. Better if outdoor obstacle avoidance at speed is the priority. |
| **Luxonis OAK-D Pro W** | Stereo + onboard VPU | Runs detection *on the camera*, offloading the Jetson. Attractive when your 8 GB is under pressure. Smaller ecosystem for SLAM. |
| **Stereolabs ZED 2i / ZED X** | Passive stereo, neural depth | Excellent outdoor range and quality. ZED X needs GMSL capture hardware; the depth pipeline is GPU-hungry, competing with your detection model. |
| **Orbbec Gemini 335 / 336L** | Active stereo | Solid ROS 2 support, competitive pricing, credible RealSense alternative. |

**Recommendation:** D435i when you reach milestone 6. It is the path of least resistance for
both `nvblox` (local mapping) and `cuVSLAM` (VIO), which is the whole reason you bought a Jetson.

### Passive vs. active, outdoors — the three classes

Any depth camera you buy must produce usable depth from **passive stereo** outdoors. The
distinction that decides this:

| Class | Mechanism | Outdoors? | Examples |
|---|---|---|---|
| **Pure active** — *"Kinect-class"* | Structured light or time-of-flight. No projector → no depth at all | ❌ Blinded by sun | Kinect v1/v2, Azure Kinect, Orbbec Femto Bolt |
| **Active-assisted stereo** | Stereo core **plus** an optional IR projector | ✅ Projector dies, **stereo carries on** | RealSense D435i / D455 |
| **Pure passive stereo** | Two cameras, natural texture only | ✅ Sunlight *helps* | Stereolabs ZED 2i |

**Why active sensors fail outdoors:** they detect *their own* projected IR. Sunlight delivers
~1000 W/m² of broadband radiation with a large near-IR component; a projector emits a few hundred
milliwatts. Signal-to-noise collapses and usable range falls from metres to near zero in direct
sun.

**Why the D435i is still fine:** it has a **fallback**. Its projector merely adds artificial
texture to help on blank indoor walls. Outdoors the projector is useless but the stereo core keeps
working on natural texture. A pure active sensor has no fallback — remove the projected pattern
and there is nothing left to measure.

Passive stereo actively *benefits* from sun: more light means better images and more texture
contrast to match on. This is why every outdoor drone depth solution is stereo-based.

> **"Kinect-class"** refers to the category the Microsoft Kinect (2010) defined — the first
> mass-market depth camera, which made real-time depth a $150 consumer part instead of a
> multi-thousand-dollar instrument and effectively started modern RGB-D robotics research. For
> this aircraft it is disqualified twice: sunlight blindness, and mass (an Azure Kinect DK is
> ~440 g against ~600 g of spare payload).

## Camera loadout: interfaces, and wide + narrow

### Does a stereo camera use the CSI lanes? — No

| Camera | Interface | CSI connectors used |
|---|---|---|
| RealSense D435i / D455 | USB3 | **0** — stereo matching runs on its internal ASIC |
| OAK-D series | USB3 | **0** — Myriad X VPU |
| ZED 2i | USB3 | **0** |
| ZED X | GMSL (capture card) | 0 |
| **Raw stereo pair** (2× IMX219 etc.) | CSI | **2** — one per imager, you do the matching |
| Single CSI camera | CSI | 1 |

**The common drone depth cameras leave both CSI connectors free.**

### The Orin Nano dev kit's two connectors are not identical

| Connector | Capability |
|---|---|
| **CAM0 (J20)** | 2-lane only — clock limitation |
| **CAM1 (J21)** | 2-lane **or 4-lane** |

Put the higher-resolution / higher-framerate camera on **J21**.

### Resulting loadout — all three coexist

```
CSI J20 (2-lane) ──► wide RGB      — navigation, search, detection
CSI J21 (4-lane) ──► narrow RGB    — detail capture
USB3             ──► D435i depth   — M7, obstacle avoidance
```

### Why wide + narrow

It maps directly onto the [08](08-mission-data-flow.md) workflow — wide finds contacts in search,
narrow captures detail in investigate. Detail vs standoff at 1080p:

| FOV | Coverage at 50 m | Ground sample | 1.5 m animal |
|---|---|---|---|
| 90° (wide) | 100 m | 52 mm/px | ~29 px |
| 40° | 36 m | 19 mm/px | ~79 px |
| **20°** | 17.6 m | 9.2 mm/px | **~163 px** |
| 10° | 8.8 m | 4.6 mm/px | ~326 px |

⭐ **A 20° lens at 50 m gives the detail a 90° lens gives at 9 m — about 5.5× the standoff.**

Rough pixels-on-target guidance: ~3 px to detect presence, ~10 px to classify, ~20+ px to
identify individuals or read behaviour.

### ⚠ Narrow FOV probably needs a gimbal

At 20° and 50 m the frame is 17.6 m wide, so a **5° attitude error swings the view 4.4 m** — a
quarter of the frame. A multirotor holding position in wind changes attitude by several degrees
continuously. A fixed narrow camera will spend much of its time pointed at nothing.

A 2-axis gimbal decouples pointing from airframe attitude but costs **~100–200 g** — a third of the
spare payload — plus mounting complexity.

Narrow FOV magnifies **motion blur and vibration** proportionally. This makes the "two envelopes"
point in [09](09-performance-envelope.md) concrete: the usable flight envelope with a 20° lens is
meaningfully tighter than with a 90° one.

**Search rate also collapses** at narrow FOV — which is exactly why the wide camera stays.

### Wildlife-specific notes

**Thermal is the standard tool for wildlife survey**, not RGB. Animals are warm and foliage is
not, so detection works through partial cover and at night. FLIR Lepton ~$200 (160×120, low res);
Boson 640 ~$3,000. The eventual upgrade for this mission, not a starting purchase.

⚠ **Check the legal position before flying near animals.** Drones demonstrably disturb wildlife
and it is regulated: national parks ban drones outright, harassing marine mammals is a federal
offense under the MMPA, and endangered-species and state harassment laws apply. Standoff is a
compliance question here, not only an image-quality one.

⚠ **Compute:** two RGB streams plus depth on an 8 GB Orin Nano with no hardware encoder is a lot.
Run detection continuously on the **wide** camera only and pull narrow-camera frames on demand
during phase 3 — which is what [08](08-mission-data-flow.md) already specifies.

## Sensors beyond the camera

| Sensor | Purpose | Verdict |
|---|---|---|
| Downward **rangefinder** (TFmini/Lightware) | Accurate altitude near ground, precise landing | **Buy early.** Cheap, huge reliability gain on takeoff/landing. |
| Downward **optical flow** (PMW3901) | Position hold without GPS at low altitude | Cheap insurance for the GPS-denied goal. Not a substitute for VIO. |
| **RTK GNSS** | cm-level position | Only if mapping/photogrammetry accuracy becomes a real requirement. Adds a base station and cost. |
| **Lidar** (Livox Mid-360 etc.) | Dense 3D mapping, robust avoidance | Heavy, expensive, and overkill for a first testbed. Revisit if goal 4 becomes primary. |

## Monocular depth estimation — what it is good for, and what it is not

### How it works

A single image is **geometrically ambiguous**: a small near object and a large far one project
identically. Depth cannot be recovered from one frame by geometry. These models instead learn
*priors* — cars are about this big, texture fines with distance, perspective converges, haze grows
with range. The cues you use with one eye closed.

Trained supervised against lidar ground truth (KITTI, NYU), self-supervised from video via
structure-from-motion consistency, or from stereo pairs.

**Current SOTA (2026):** **Depth Anything 3** — its metric variant substantially outperforms
UniDepth v2 on outdoor benchmarks — alongside **Depth Pro** and **UniDepth v2**. ROS 2 TensorRT
nodes for DA3 exist, so this is deployable on the Orin Nano today.

### The crux: scale

| Output | Meaning | Reliability |
|---|---|---|
| **Relative depth** | "this pixel is nearer than that one" | Robust — what these models genuinely do well |
| **Metric depth** | "this pixel is 4.2 m away" | Much harder — **a single image contains no scale information** |

A real kitchen and a dollhouse kitchen are indistinguishable. "Metric" models have memorized
typical scales for typical scenes; they degrade sharply off-distribution.

### ❌ Why it is weak for obstacle avoidance

1. **Scale ambiguity is disqualifying.** "Nearer than background" does not tell you whether to
   brake. You need "wall at 3.2 m, closing at 5 m/s."
2. **Aerial views are out of distribution.** Training data is overwhelmingly ground-level; a 20 m
   oblique view is not, and accuracy drops badly on unseen scene types.
3. **Thin structures are missed** — power lines, guy wires, branches, fences: exactly what kills
   drones. The UAV literature is explicit that fine features are underrepresented in datasets and
   that monocular interpolation lacks the accuracy to avoid them.
4. **No failure signal.** Stereo returns *invalid* pixels when it cannot match; a network always
   returns a confident number. You cannot distinguish "I know" from "I am guessing" — and a
   confident wrong answer is worse than none in a safety-critical loop.
5. **Compute contention.** It competes with the detector for the same 8 GB and GPU. Stereo can run
   on dedicated silicon (RealSense ASIC, OAK VPU) and offload the Jetson entirely.

### ✅ Where it *is* useful here

**Improving detection geolocation — the real application.** [08](08-mission-data-flow.md)
geolocates contacts by intersecting a camera ray with an **assumed flat ground plane**, and that
assumption is the dominant error term over sloping terrain. A monocular depth map supplies
relative terrain shape; the downward rangefinder supplies the metric anchor that resolves scale.
Together they could meaningfully tighten the 2–10 m geolocation error.

**And it is not safety-critical in that role.** A bad estimate moves a map pin; it does not fly
you into a tree. That asymmetry is what makes it appropriate here and inappropriate for avoidance.

Also viable: **densifying photogrammetry** offline for goal 4, and **fusing with stereo** — stereo
where texture exists, monocular filling textureless regions.

### ⭐ The distinction worth remembering

**Single-frame monocular depth is a learned prior with ambiguous scale. Monocular depth from
*motion* is real geometry.**

A moving camera produces parallax — a stereo baseline over time. That is measurement, not
inference, and it is how ORB-SLAM and cuVSLAM work. Pure monocular SLAM still has scale ambiguity,
but **the IMU resolves it** — precisely why it is visual-*inertial* odometry. Active 2026 research
targets exactly this: zero-shot metric depth from monocular visual-inertial input.

So the single-camera capability you would actually want is already planned as **M8 (cuVSLAM)**, and
it works for reasons single-frame depth networks do not.

## The GPS-denied question (deferred, not foreclosed)

You answered "not sure yet" on indoor vs. outdoor. The design consequence:

- **Build for outdoor/GNSS now.** GPS + compass + baro gives you a solid EKF for free.
- **Keep the door open** by (a) choosing a camera with a synchronized IMU and global shutter,
  (b) leaving a spare USB3/CSI lane and ~100 g of payload margin, (c) using PX4, whose EKF2
  accepts external vision position estimates through a well-trodden path.
- When you want indoor: add `isaac_ros_visual_slam` (cuVSLAM), publish its pose into PX4 as an
  external vision estimate, and fuse. That is a software change plus one camera, not a redesign.

This is the cheapest way to defer the decision without painting yourself into a corner.

## Sources

- [RealSense completes spin-out from Intel, raises $50M](https://www.intelcapital.com/realsense-completes-spin-out-from-intel-raises-50-million-to-accelerate-ai-powered-vision-for-robotics-and-biometrics/)
- [RealSense spinout + NVIDIA collaboration](https://www.businesswire.com/news/home/20251008178223/en/RealSense-Spins-Out-from-Intel-Secures-$50-Million-and-Announces-Strategic-Collaboration-With-NVIDIA-to-Accelerate-Physical-AI-and-Robotics)
- [Best depth cameras for robotics 2026 — RealSense vs ZED vs Orbbec](https://www.roboticscenter.ai/blog/best-depth-cameras-robotics)
- [RealSense: depth map improvements for drones](https://dev.realsenseai.com/docs/depth-map-improvements-for-stereo-based-depth-cameras-on-drones/)
- [Depth Anything 3: Recovering the Visual Space from Any Views](https://arxiv.org/html/2511.10647v1)
- [Survey on Monocular Metric Depth Estimation](https://arxiv.org/abs/2501.11841)
- [Monocular Depth Estimation for UAV Obstacle Avoidance](https://ieeexplore.ieee.org/document/8980350)
- [DDOS: Drone Depth and Obstacle Segmentation Dataset — thin obstacles](https://arxiv.org/html/2312.12494v1)
- [Zero-Shot Metric Depth via Monocular Visual-Inertial](https://arxiv.org/pdf/2509.08159)
- [Using the Jetson Orin Nano with CSI cameras — JetsonHacks](https://jetsonhacks.com/2023/04/05/using-the-jetson-orin-nano-with-csi-cameras/)
- [OAK vs RealSense comparison — Luxonis](https://docs.luxonis.com/hardware/platform/comparison/vs-realsense)

---

## Glossary

| Term | Meaning |
|---|---|
| **CPU** | Central Processing Unit — the general-purpose processor cores. |
| **CSI** | Camera Serial Interface (MIPI CSI) — a direct ribbon-cable camera connection. Lower latency and CPU cost than USB. |
| **cuVSLAM** | NVIDIA's GPU-accelerated visual SLAM implementation, shipped as part of Isaac ROS. |
| **EKF** | Extended Kalman Filter — the math that fuses IMU, GPS, compass and barometer into a single position and attitude estimate. EKF2 is PX4's version, EKF3 ArduPilot's. |
| **FOV** | Field Of View — how wide an angle the camera sees. |
| **gimbal** | A motorized mount that keeps a camera steady and pointed as the aircraft moves. |
| **GMSL** | Gigabit Multimedia Serial Link — automotive-grade camera cabling; needs a dedicated capture board. |
| **GNSS** | Global Navigation Satellite System — the general term for satellite positioning. GPS is the US one. |
| **GPS** | Global Positioning System — satellite positioning. |
| **GPU** | Graphics Processing Unit — on a Jetson, the part that runs the vision models. |
| **IMU** | Inertial Measurement Unit — accelerometers plus gyroscopes. Senses motion and orientation. |
| **IR** | Infrared — light just beyond human vision, used by depth-camera projectors. Washed out by sunlight. |
| **MIPI** | The industry group behind CSI, the direct camera-connection standard. |
| **nvblox** | NVIDIA's GPU library that builds a live 3D map from depth images. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RGB** | Red Green Blue — an ordinary color camera, as opposed to a depth camera. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **RTK** | Real-Time Kinematic — a GPS technique giving centimetre accuracy using a fixed base station. |
| **SLAM** | Simultaneous Localization And Mapping — building a map while simultaneously working out where you are in it. |
| **TensorRT** | NVIDIA's library that optimizes a trained neural network to run fast on a specific GPU. |
| **UAV** | Unmanned Aerial Vehicle. UAS (Unmanned Aircraft System) is the formal regulatory term. |
| **USB3** | USB 3.0 — fast USB, and a notorious source of 2.4 GHz radio noise. |
| **VIO** | Visual-Inertial Odometry — estimating motion by combining camera images with IMU data. How you navigate without GPS. |
| **VPU** | Vision Processing Unit — a chip specialized for vision work, e.g. inside an OAK-D camera. |
| **ZED** | Stereolabs' line of stereo depth cameras. |
