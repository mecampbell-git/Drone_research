# 05 — Software Stack

## The stack

```
  mission logic  (your Python/C++ ROS 2 nodes)
  ────────────────────────────────────────────
  Isaac ROS 4.6   cuVSLAM · nvblox · TensorRT DNN inference · NITROS zero-copy
  ────────────────────────────────────────────
  ROS 2 Jazzy     (Ubuntu 24.04)
  ────────────────────────────────────────────
  JetPack 7.2     L4T r39.2 · CUDA 13.2.1 · cuDNN 9.20 · TensorRT 10.16.2
  ────────────────────────────────────────────
  Jetson Orin Nano
        ║  uXRCE-DDS over UART
  ────────────────────────────────────────────
  PX4 (NuttX)  →  Pixhawk 6C
```

## Flight stack: PX4 or ArduPilot?

Both are mature and both now speak ROS 2 natively — the old "PX4 has DDS, ArduPilot only has
MAVROS" claim is out of date. **ArduPilot has native DDS via `AP_DDS` from 4.5+**, using the same
XRCE-DDS protocol.

| | PX4 | ArduPilot |
|---|---|---|
| ROS 2 bridge | uXRCE-DDS, mature, `px4_ros_com` | AP_DDS (4.5+), newer, smaller topic coverage |
| Offboard control | First-class, heavily used | Supported (GUIDED mode) |
| External vision / VIO into EKF | EKF2, well-trodden | EKF3, also supported |
| Vehicle types | Focused | Enormous (rovers, boats, planes, subs…) |
| Docs for *this* use case | Strongest | Strong but more scattered |
| Community tone | Research / commercial | Hobbyist / very broad |

**Choose PX4.** Not because ArduPilot is worse in general, but because for *"Jetson companion
computer streaming offboard setpoints over ROS 2"* specifically, PX4 has the deepest documentation,
the most examples, and the tightest ROS 2 integration. The Holybro kit ships PX4-flashed. This is
a reversible decision — the same Pixhawk 6C runs either firmware.

## Versions — pinned by the JetPack already on the device

**This project's Jetson runs JetPack 7.2.** That fixes most of the stack:

| Layer | Version |
|---|---|
| JetPack | **7.2** (Jetson Linux L4T **r39.2**) |
| OS | **Ubuntu 24.04** |
| CUDA | 13.2.1 |
| cuDNN | 9.20 |
| TensorRT | 10.16.2 |
| **ROS 2** | **Jazzy** — the native distro for 24.04 |
| **Isaac ROS** | **4.6** — the release that supports Orin on JetPack 7.2 with Jazzy |

⚠ **Earlier drafts of this document specified JetPack 6.x / Ubuntu 22.04 / ROS 2 Humble. That is
wrong for this hardware** and is corrected here. Humble belongs to the older Isaac ROS 3.2
generation; **do not mix a 4.6 workspace with Humble dependencies from a 3.2 system.**

### Stay on 7.2 — do not downgrade

It is already installed, Isaac ROS 4.6 officially supports it, and CUDA 13 / TensorRT 10.16 are
meaningfully newer.

#### Considered and rejected: downgrading to JetPack 6.2 for ROS 2 Humble

**The case for it:** Isaac Sim on Windows supports **only Humble** (still true in Isaac Sim 6.0). A
Humble Jetson would allow M7 ROS-in-the-loop simulation on Windows with no version seam. Humble also
has more tutorials, and PX4's docs lead with it.

**Why not:**

| Cost | Detail |
|---|---|
| **Humble EOL: May 2027** | ~8 months out. The roadmap outlives it, forcing a migration mid-project — likely around M5–M7, the hardest stretch. Jazzy is supported to **May 2029**. |
| **Legacy Isaac ROS line** | Humble means Isaac ROS 3.2. New cuVSLAM / nvblox / TensorRT-node work is on 4.x + Jazzy — the packages that justify the Jetson. |
| **Older CUDA / TensorRT** | JetPack 6.2: CUDA 12.6, TensorRT 10.3 — vs 13.2 / 10.16 on 7.2. |
| **The downgrade is a full reflash** | NVIDIA: requires a complete **SDK Manager** flash from an Ubuntu host, which **erases the NVMe** and must also restore the **QSPI boot firmware** (7.2 upgraded it). Writing a 6.2 image to storage alone will not boot correctly, and forum reports show people stuck on the firmware step. |
| **Dev box moves to Ubuntu 22.04** | To match the target, since Humble's official binaries are on 22.04. |

**The deciding factor:** the only real argument for Humble was avoiding Linux on the 5070 Ti
machine — and that machine is now the Ubuntu dev box, with the risk removed. Driving the display from the **motherboard HDMI is
confirmed working** on that machine, with zero VRAM used by the desktop. Isaac Sim 6.0's own docs
recommend **Ubuntu 24.04 with Jazzy**. So M7 runs end-to-end on Jazzy (see "Isaac Sim at M7"
below).

Humble tutorials are not a blocker: beginner-level ROS 2 concepts and APIs are nearly identical,
and usually only the distro name in install commands changes.

The cost is that Jazzy is less-trodden than Humble: more tutorials, forum answers and third-party
packages still assume Humble. That gap is closing, and none of it is blocking — but verify the
following **early, at M0 and M2.5, before building anything on the assumption**:

| Verify | Why |
|---|---|
| `px4_msgs` / `px4_ros_com` build on **Jazzy** | The core FC integration. PX4 docs still lead with Humble. |
| **Python 3.12** wheels for ARM64 | Ubuntu 24.04 ships Python 3.12 (22.04 had 3.10). Some packages lag, and this is a common source of "works on my desktop" failures. |
| `realsense-ros` Jazzy branch | Needed at M7, not before. |
| Isaac ROS container tags | NVIDIA's own catalog pages have carried Humble/Jazzy labelling inconsistencies — check the release notes, not the catalog blurb. |

### TensorRT 10.16 — rebuild your engines

Engines are tied to both GPU architecture **and** TensorRT version. Anything exported against
TensorRT 8.x will not load. Export ONNX, then build the engine **on the Jetson itself** under
10.16.2. This is the same portability trap described in the fidelity ladder below — JetPack 7.2
just makes the version half of it sharper.

## Isaac ROS — the reason for the Jetson

The packages that map directly onto your four goals:

| Package | Gives you | Goal |
|---|---|---|
| `isaac_ros_dnn_inference` / TensorRT | Hardware-accelerated detection (YOLO-class, exported to TensorRT engine) | 1, 2 |
| `isaac_ros_visual_slam` (cuVSLAM) | GPU visual-inertial odometry | GPS-denied |
| `isaac_ros_nvblox` | Live 3D reconstruction → ESDF cost map for planning | 3, 4 |
| `isaac_ros_image_proc` | GPU rectification, resize, format conversion | all |
| **NITROS** | Zero-copy GPU-to-GPU between nodes — avoids the CPU round-trips that would eat your 8 GB and your latency | all |

**Use NITROS-enabled node chains.** Naive ROS 2 image pipelines copy full frames through CPU
memory several times per hop; on an 8 GB shared-memory device that is the difference between
30 fps and 8 fps.

## Models

- Start with an off-the-shelf **YOLO-family** detector, exported ONNX → TensorRT engine, **INT8
  quantized**. Build the engine *on the Jetson itself* — TensorRT engines are not portable across
  devices or TensorRT versions.
- Measure end-to-end latency (capture → detect → setpoint), not just inference time. The camera
  pipeline and ROS transport usually dominate.
- Fine-tune on your own flight footage later. This is why you log raw video to NVMe.
- A vision-language model on-board is feasible at this TOPS level but is a milestone-8 luxury,
  not a starting point.

## Development environment — machine allocation

**Decision: the RTX 5070 Ti machine is dual-booted with Ubuntu 24.04 and used for everything.**
It is the newest, fastest machine, and driving the display from the iGPU removes most of the
Blackwell-on-Linux risk.

| Machine | Role | OS |
|---|---|---|
| **RTX 5070 Ti (16 GB Blackwell)** | **Everything** — PX4 SITL, Gazebo, ROS 2 Jazzy, HITL host, model fine-tuning, Isaac Sim (M7) | **Ubuntu 24.04** (dual-boot with Windows) |
| **Jetson Orin Nano** | Inference, on the aircraft | Ubuntu 24.04 (JetPack 7.2) |
| **Laptop** | Field ops — QGroundControl, SSH to the Jetson | Omarchy, unchanged |
| i7-8700, 16 GB | Spare — no role required | — |

### Display on the iGPU — confirmed working

**The monitor is connected to motherboard HDMI**, tested on this machine with **zero VRAM used by
the desktop**. The NVIDIA card does compute and Isaac Sim rendering only. This:

- avoids the installer black-screen problem entirely
- turns a bad NVIDIA driver install into "working desktop, try again" rather than a black screen
- keeps all 16 GB of VRAM free — which matters, since 16 GB is exactly Isaac Sim 5.1's minimum

### Why Ubuntu 24.04, and not Arch

**Match the target.** The Jetson runs Ubuntu 24.04. A dev machine on the same distro shares OS,
package versions and ROS binaries, leaving **CPU architecture as the only difference** — what
makes the byte-identical-binary discipline hold.

ROS 2 Jazzy has **official Tier 1 binaries on Ubuntu 24.04** and only community AUR source builds on
Arch; PX4 ships `ubuntu.sh`; Gazebo is an apt package; Isaac ROS containers are Ubuntu-based; Isaac
Sim 6.0 recommends Ubuntu 24.04 with Jazzy. Keep Omarchy on the laptop for field work —
QGroundControl ships as an AppImage and runs anywhere.

### ⭐ Gazebo does not need a GPU

Physics is CPU-only, PX4 SITL runs headless (`HEADLESS=1`), and rendering is only needed for
simulated cameras. **M0–M4 need no GPU at all.** The 5070 Ti earns its keep at M7 (Isaac Sim) and
for fine-tuning.

### Model fine-tuning

PyTorch with CUDA runs on the Ubuntu side, so **no reboot into Windows is needed to train**. Train
→ export **ONNX** → build the TensorRT engine **on the Jetson** (required anyway: engines are
locked to GPU architecture and TensorRT version).

### Isaac Sim at M7

Runs on this machine's Ubuntu install with **ROS 2 Jazzy** — an all-Jazzy chain: Isaac Sim →
perception nodes → Jetson. (Isaac Sim on Windows supports only Humble, which is why the Linux
side is used.) If Isaac Sim proves troublesome, skipping it is genuinely viable — test obstacle
avoidance outdoors with real sensors and prop guards.

### ⚠ Dual-boot — do these before touching anything

| Step | Why |
|---|---|
| **Save the BitLocker recovery key** ([account.microsoft.com/devices/recoverykey](https://account.microsoft.com/devices/recoverykey)), stored off the machine | Windows 11 often enables device encryption silently. Disabling Secure Boot or changing boot config can trigger a recovery-key prompt — without the key, the Windows drive is inaccessible. |
| **Disable Windows Fast Startup** (Control Panel → Power Options → "Choose what the power buttons do") | It leaves the Windows volume half-hibernated; Linux touching it can corrupt it. |
| **Separate SSD for Ubuntu if possible** | Safest dual-boot: separate bootloaders, Windows partition untouched. Otherwise shrink the Windows partition *from Windows* (Disk Management) before installing. |

### Setup order

1. BitLocker key saved, Fast Startup off, space prepared (second SSD, or Windows partition shrunk)
2. BIOS: **iGPU as primary display**, **Secure Boot off** (simplest)
3. Install the **newest Ubuntu 24.04 point release** — not 25.x — monitor on motherboard HDMI
4. `sudo apt install nvidia-driver-580-open`, reboot, verify with `nvidia-smi`
5. ROS 2 Jazzy, then PX4 + Gazebo, then (later) Isaac Sim

**Partition: ≥500 GB.** Isaac Sim is ~20–30 GB, Isaac ROS containers multi-GB each, and `ros2 bag`
files grow fast.

**`git init` the project repo** (Q7 in [open-questions.md](open-questions.md)) so docs and the ROS
workspace are reachable from both OSes.

Xorg over Wayland remains the safer default for Gazebo and Isaac Sim GUIs, though far less fraught
with the desktop on the iGPU.

---

### ⚠ Blackwell (RTX 50 series) on Linux — install gotchas

*Relevant from day one — the 5070 Ti machine is the Ubuntu dev box.* **None of this applies on
Windows** — the normal Windows NVIDIA driver fully supports Blackwell, so fine-tuning and Isaac Sim
just work there. Real issues on Linux, all with known fixes. Budget **an evening**, not a weekend.

#### ⭐ Best mitigation: drive the display from the iGPU

Plug the monitor into the **motherboard**, and use the NVIDIA card for compute only. The `-open`
driver is still required (CUDA needs the kernel module either way), but most Blackwell pain is
*display* pain:

| Problem | With display on iGPU |
|---|---|
| Installer black screen | **Gone** — installer uses the iGPU, no `nomodeset` |
| Black screen after a bad driver install | **Gone** — working desktop to fix it from |
| NVIDIA + Wayland/Xorg compositor quirks | **Gone** — compositor on the Intel/AMD open driver |
| Wrong package (`-open` vs not), Secure Boot signing, driver 580+ | Still apply |

**A driver failure goes from "black screen, reinstall" to "working desktop, try again."** Bonus:
the desktop stops using VRAM, which matters when 16 GB is exactly Isaac Sim's minimum.

Caveats:
- **The CPU must have an iGPU.** Intel "F" SKUs and AMD Ryzen 5000 non-G chips do not; Ryzen
  7000/9000 do.
- **BIOS may disable the iGPU** when a discrete card is present — look for "Primary display: IGD"
  or "iGPU multi-monitor".
- **Fine-tuning: no downside.** Pure CUDA.
- **Isaac Sim GUI: some friction.** It renders on the NVIDIA card, so showing it on an iGPU display
  uses PRIME render offload (`__NV_PRIME_RENDER_OFFLOAD=1`). Alternatively run it headless and view
  via NVIDIA's WebRTC streaming client, which reportedly still wants a virtual display (`xvfb`).

#### The critical one: use the `-open` driver

**NVIDIA's proprietary driver branch does not support RTX 50 series at all.** Blackwell requires
the open kernel modules:

```bash
sudo apt install nvidia-driver-580-open      # NOT nvidia-driver-580
```

Installing the non-open package is the most common failure and presents as a **black screen or
`nvidia-smi` failing**, not as an obvious error. If using `ubuntu-drivers autoinstall`, verify what
it selected.

**580+ is the minimum** for 50-series; Isaac Sim 5.1 wants 580.65.06+ on Linux, so these align.

#### Install-time traps

| Trap | Fix |
|---|---|
| **Installer black-screens** — nouveau in older kernels lacks Blackwell PCI IDs | At GRUB press `e`, add `nomodeset` to the `linux` line, Ctrl-X. ~720p, no acceleration, fine for installing. Remove once the driver is in. |
| **Old ISO** — 24.04 GA shipped kernel 6.8 | Use the **newest 24.04 point release** (24.04.3+) for the HWE kernel. Do not reuse an old ISO. |
| **Secure Boot** — unsigned modules silently fail to load | **Disable it in BIOS** before installing. If left on, enroll the MOK at the blue screen on first reboot — missing that screen looks identical to a failed install. |
| **Mixed install methods** | apt **or** the `.run` installer, never both. Mixing them is the classic unfixable-without-reinstall scenario. |

**If the CPU has an iGPU, that is the rescue path** — install using integrated graphics output, then
add the NVIDIA driver. This is the documented workaround for the worst RTX 5090 cases.

Verify with `nvidia-smi` before moving on.

#### Do not "solve" this with a newer Ubuntu

Ubuntu 25.x has newer kernels but is **not LTS and not what ROS 2 Jazzy targets**. That trades a
one-evening driver problem for months of package friction. **Stay on 24.04 LTS.**

### Sequencing note

**M0–M6 need only Gazebo**, which is light. Isaac Sim first earns its place at **M7** for
obstacle-avoidance testing in cluttered photorealistic scenes. Nothing about the Ubuntu move needs
to wait for that — but nothing is blocked if Isaac Sim setup is deferred either.

## Simulation — two tools, two different jobs

**Gazebo tests whether the aircraft goes where you told it. Isaac Sim tests whether the camera
sees what you think it sees.** They are not alternatives.

### PX4 SITL + Gazebo — the control half

**SITL means the real PX4 firmware runs on your PC** — the same C++ that would run on the STM32
in your Pixhawk, compiled for x86. You are not testing a *model* of PX4; you are testing PX4:
real EKF, real controllers, real mode logic, real failsafes, real parameters, real MAVLink.

Gazebo supplies the world:

```
PX4 ─► motor commands ─► Gazebo applies forces, integrates rigid-body physics
    ─► simulated IMU / GPS / baro / mag (with noise) ─► PX4's EKF ─► repeat
```

**Good for:** flight dynamics, mode transitions, mission logic, failsafe behavior, correctness of
your offboard control node. Anything phrased *"does the aircraft do the right thing?"*

**Bad at:** looking real. Rendering is functional programmer-art — flat textures, simple
lighting. A detector tuned on Gazebo imagery tells you almost nothing about real performance.

**Runs on anything**, no GPU needed, especially headless. Free. Start M0 today.

### Isaac Sim — the perception half

NVIDIA's simulator on Omniverse/USD with **RTX ray-traced photorealistic rendering** —
physically-based materials, real lighting, shadows, reflections.

**Good for:** does the detector actually detect? How does it degrade at dusk, in fog, at a
different sun angle? And **synthetic training data with perfect ground-truth labels for free** —
bounding boxes, segmentation masks, depth, all exact — plus domain randomization over lighting,
textures and placement to make models robust.

It has physics (PhysX) and a PX4 bridge, so it *can* replace Gazebo. It is simply much heavier
for that job.

**Costs:** a capable RTX GPU on the desktop (see Q3 in [open-questions.md](open-questions.md)),
slower iteration, more setup complexity.

### Which tool for which question

| Question | Tool |
|---|---|
| Does my offboard node fly the square? | **Gazebo** |
| What happens if the companion computer dies mid-mission? | **Gazebo** |
| Does RTL trigger at the right battery level? | **Gazebo** |
| Is my geofence configured correctly? | **Gazebo** |
| Does my detector find cones at 20 m in afternoon light? | **Isaac Sim** |
| How does the model behave at dusk or in fog? | **Isaac Sim** |
| I need 10,000 labeled training images | **Isaac Sim** |
| Does nvblox build a sane map of a cluttered room? | **Isaac Sim** |

### The distinction that saves the most time

Gazebo *does* render camera images — just not realistically. That makes it **perfectly adequate
for testing perception plumbing**: does my node subscribe to the right topic, does the TF chain
produce correct geolocation, does the detection → setpoint path close?

**You need Isaac Sim when *appearance* matters, not when *wiring* matters.** Most perception bugs
are wiring bugs, and Gazebo finds those for free.

⚠ Isaac Sim is not a substitute for real data. The sim-to-real gap is real. Use synthetic data
to *augment* real footage, or to cover cases you cannot safely collect — never as the only
training set. Your own M3/M5 flight footage will likely be worth more than synthetic imagery for
your specific mission.

### ⚠ SITL does not simulate the Jetson

A common misconception worth killing early. In pure SITL **everything runs on your desktop**:

```
Your desktop:
  ├── PX4 SITL          (real PX4 firmware, compiled for x86)
  ├── Gazebo            (physics + simulated sensors)
  └── Your ROS 2 nodes  (the SAME code that will later run on the Jetson)
```

The Jetson *hardware* is not modelled. Your Jetson *software* simply runs natively on the
desktop, talking to PX4 SITL over uXRCE-DDS exactly as it will talk to the real Pixhawk over
UART — same topics, same messages, same API. That is why the code transfers unmodified.

**Transfers perfectly:** control logic, state machines, mission sequencing, ROS 2 plumbing, the
TF chain, failsafe handling — anything answering *"what command do I send next?"*

**Does not transfer:**

| Gap | Why it matters |
|---|---|
| **Compute performance** | Desktop is far faster. A detector at 60 fps on a desktop RTX may manage 12 fps on an Orin Nano. SITL tells you nothing about real latency. |
| **CPU architecture** | Desktop x86_64 vs Jetson ARM64. Compiled deps and CUDA code are not portable. **TensorRT engines must be built on the Jetson** — they are tied to a specific GPU architecture and TensorRT version. |
| **Memory** | 32–64 GB desktop vs **8 GB shared** on the Jetson. Code that runs fine on desktop can OOM on target. |
| **Camera** | Gazebo's simulated camera vs a real driver, real timing, real bandwidth, real exposure. |
| **FC link** | UDP on localhost vs a real 921600-baud UART with real latency and dropouts. |
| **Thermal throttling** | Does not exist in SITL. |

**SITL validates logic. Hardware validates performance.** They catch disjoint sets of bugs.

### The fidelity ladder

Four rungs, each adding one class of realism. Climb them in order:

| Rung | Milestone | Setup | Adds | Risk |
|---|---|---|---|---|
| **1. Pure SITL** | **M0** | Everything on desktop | Control logic, mission state machines | none |
| **2. Jetson + SITL** | **M2.5** | Nodes on the **real Jetson**, PX4 SITL on desktop, over the network | **Real compute performance, real ARM64 build, real TensorRT engine** | none |
| **3. HITL** | **M3.5** | Real Pixhawk + real Jetson, simulated world | **Real UART link under load**, real FC timing, real boot behaviour, **rehearsed RC override** | none |
| **4. Flight** | **M4** | Everything real | Vibration, thermal, wind, EMI, consequences | real |

**Rung 2 is the one people skip and regret.** It is where you discover your detector runs at
12 fps instead of 60, or that a dependency has no ARM64 wheel — while crashes still cost nothing.
It sits naturally between M0 and M4 in [06](06-autonomy-roadmap.md).

One caveat on rung 2: streaming Gazebo's simulated camera over the network to the Jetson adds
latency that is not representative of a real camera. Use it to measure *inference* throughput,
not end-to-end glass-to-setpoint latency.

### Hardware-in-the-loop (HITL) — how it actually works

HITL runs **the real PX4 firmware on the real Pixhawk**, but feeds it fake sensors.

```
     Desktop: Gazebo Classic
        │  physics → simulated IMU, baro, mag, GPS
        │
        │  HIL_SENSOR / HIL_GPS   (MAVLink over USB)
        ▼
   REAL Pixhawk 6C — real PX4 firmware, real STM32H7, real timing
        │  EKF consumes them as if genuine → normal control loops
        │
        │  HIL_ACTUATOR_CONTROLS   (MAVLink back over USB)
        ▼
     Gazebo applies them to physics, integrates, repeats
```

Set `SYS_HITL`, select the **HIL Quadcopter X** airframe, and PX4 stops reading its own sensors
entirely. The `pwm_out_sim` module replaces the real PWM driver, so **no signal reaches the ESC
pins** — motor outputs go to the simulator instead.

| Real | Simulated |
|---|---|
| PX4 firmware, ARM build | IMU, gyro, mag, baro |
| STM32H7 timing and scheduling | GPS |
| Parameter storage, boot behaviour | Aerodynamics and physics |
| **The TELEM2 UART to the Jetson** | Motor response |
| **RC receiver and your transmitter** | |

#### Why it earns its place — the two bold rows

**The companion↔FC link.** In SITL your node talks to PX4 over localhost UDP: infinite bandwidth,
zero latency. In HITL you wire the Jetson to TELEM2 at 921600 and exercise the **actual UART** —
real latency, real bandwidth ceiling, real dropouts, real uXRCE-DDS behaviour over a constrained
serial link. That interface is otherwise untested until you are airborne at M4.

**The safety-pilot override.** Bind the real transmitter and confirm the mode switch yanks the
aircraft out of Offboard — real receiver, real board, simulated aircraft. **You get to rehearse
the emergency at zero cost.**

Plus real firmware on real silicon catches bugs the x86 SITL build cannot (different compiler,
timing, float behaviour), and you see genuine boot and power-cycle behaviour.

#### The configuration worth building

```
Desktop        Gazebo Classic — physics + simulated sensors
   ↕ USB
Pixhawk 6C     real PX4 firmware, HITL mode, props off
   ↕ TELEM2 UART @ 921600            ← the real link
Jetson         your ROS 2 Jazzy nodes, real ARM build, real TensorRT
   ↕ RC receiver ← your transmitter, in your hands
```

Everything real except physics.

#### What it does *not* catch

Vibration, EMI, thermal, real IMU noise and bias, GPS multipath, ESC and motor behaviour, and the
vibration→EKF interaction behind many real-world problems. **HITL validates firmware and
interfaces, not physics.** HITL sensor rates are also capped by the USB link — lower than real
flight — so it is a poor test of EKF tuning specifically.

#### Practical caveats

⚠ **HITL uses Gazebo *Classic*, not the newer Gazebo** that current PX4 SITL defaults to. Your M0
SITL and your HITL rig may need two different simulator installs. **jMAVSim** also does HITL
(quadrotor only) and is simpler to stand up if Gazebo Classic proves awkward.

- `pwm_out_sim` must be in the firmware build — check with `pwm_out_sim status` on the MAVLink console
- QGroundControl connects over UDP; disable other AutoConnect options
- Disable `NAV_RCL_ACT` so RC-loss failsafe does not fire spuriously
- **Remove the props anyway.** `pwm_out_sim` is a software guarantee about a machine with blades on it.

The discipline that makes this project succeed: **the ROS 2 node you fly is byte-identical to the
one you tested in SITL.** Anything behaving differently between sim and hardware belongs behind an
interface, not in an `if simulation:` branch.

## Development workflow

- Cross-compile or build in Docker on the Jetson; NVIDIA ships Isaac ROS dev containers.
- Version-control the Jetson's `ros2_ws` in this repo (this project is not yet a git repo —
  `git init` it before writing code).
- SSH + `tmux` over the Jetson's WiFi for field debugging. Bring a laptop and a folding chair.
- `ros2 bag` everything on the Jetson. Offline replay of a real flight is the single most
  valuable debugging tool you will have.

## Sources

- [PX4 uXRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds)
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide)
- [PX4 Companion Computers](https://docs.px4.io/main/en/companion_computer/)
- [ArduPilot ROS 2 / AP_DDS](https://ardupilot.org/dev/docs/ros2.html)
- [AP_DDS README](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/README.md)
- [PX4 Hardware in the Loop Simulation (HITL)](https://docs.px4.io/main/en/simulation/hitl.html)
- [Isaac ROS 4.6 on Jetson Orin: JetPack 7.2 setup](https://openelab.io/blogs/learn/isaac-ros-4-6-jetson-orin-jetpack-7-2)
- [Isaac ROS release notes](https://nvidia-isaac-ros.github.io/releases/index.html)
- [JetPack SDK downloads and release notes](https://developer.nvidia.com/embedded/jetpack/downloads)
- [Isaac Sim 5.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
- [Isaac Sim ROS 2 installation — platform support](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)
- [PX4 SITL + ROS 2 + Gazebo on WSL2 Ubuntu 24.04 (setup script)](https://github.com/cSushi/px4-sitl-ros2-wsl-setup)
- [Isaac Sim on Windows bridged to ROS 2 Jazzy in WSL2](https://github.com/AydinLT00/IsaacSim_ROS2_HelloWorld)
- [ROS 2 Jazzy on Arch — AUR package](https://aur.archlinux.org/packages/ros2-jazzy)
- [Omarchy — Arch + Hyprland by DHH](https://omarchy.org/)
- [RTX 5070 on Ubuntu 24.04 — working 580-open driver guide](https://github.com/adamn1225/FIXED-NVIDIA-RTX-5070-on-Ubuntu-24.04---Working-580-open-Driver-Guide)
- [Installing RTX 5000 Blackwell drivers on Linux — guide](https://gist.github.com/jatinkrmalik/86afb07cbe6abf5baa2d29d3842aa328)
- [How to install NVIDIA RTX 50 series drivers on Ubuntu](https://spin.atomicobject.com/nvidia-rtx-50-ubuntu/)
- [Isaac Sim setup tips — headless and streaming](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_faq.html)
- [NVIDIA PRIME render offload](https://download.nvidia.com/XFree86/Linux-x86_64/440.44/README/primerenderoffload.html)
- [Isaac Sim 6.0 ROS 2 installation — platform/distro support](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_ros.html)
- [ROS 2 end-of-life dates](https://endoflife.date/ros-2)
- [Downgrade JetPack 7.2 to 6.2 — NVIDIA forums](https://forums.developer.nvidia.com/t/downgrade-jetpack-7-2-to-jetpack-6-2/379510)
- [Orin Nano Developer Kit firmware / update path](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/update_firmware.html)

---

## Glossary

| Term | Meaning |
|---|---|
| **ArduPilot** | Open-source flight control firmware; the main alternative to PX4. |
| **CPU** | Central Processing Unit — the general-purpose processor cores. |
| **CUDA** | NVIDIA's programming platform for running general computation on the GPU. |
| **cuVSLAM** | NVIDIA's GPU-accelerated visual SLAM implementation, shipped as part of Isaac ROS. |
| **DDS** | Data Distribution Service — the publish/subscribe messaging standard underneath ROS 2. |
| **DNN** | Deep Neural Network. |
| **EKF** | Extended Kalman Filter — the math that fuses IMU, GPS, compass and barometer into a single position and attitude estimate. EKF2 is PX4's version, EKF3 ArduPilot's. |
| **ESC** | Electronic Speed Controller — the board that drives one motor from the flight controller's commands. |
| **ESDF** | Euclidean Signed Distance Field — a 3D grid storing the distance to the nearest obstacle. What a path planner reads to avoid things. |
| **failsafe** | Pre-programmed automatic behavior when something goes wrong (link lost, battery low, companion computer silent). |
| **FC** | Flight Controller — the real-time board (a Pixhawk here) that actually stabilizes and flies the aircraft. |
| **geofence** | A virtual boundary. Crossing it triggers an automatic response such as RTL. |
| **GPS** | Global Positioning System — satellite positioning. |
| **GPU** | Graphics Processing Unit — on a Jetson, the part that runs the vision models. |
| **GUIDED** | ArduPilot's mode for accepting commands from a companion computer. PX4 calls the equivalent Offboard. |
| **HITL** | Hardware In The Loop — real flight controller and Jetson wired to a simulated world. |
| **IMU** | Inertial Measurement Unit — accelerometers plus gyroscopes. Senses motion and orientation. |
| **INT8** | 8-bit integer — a compressed number format that makes neural networks run much faster with a small accuracy cost. |
| **Isaac ROS** | NVIDIA's set of GPU-accelerated ROS 2 packages for robot perception. |
| **JetPack** | NVIDIA's software bundle for Jetson boards: Linux, CUDA, TensorRT and drivers. |
| **L4T** | Linux for Tegra — the Linux kernel and OS layer inside JetPack. |
| **MAVLink** | The standard lightweight messaging protocol between drones, ground stations and companion computers. |
| **MAVROS** | An older bridge that translates MAVLink into ROS messages. |
| **NITROS** | NVIDIA Isaac Transport for ROS — passes images between nodes on the GPU without copying through the CPU. |
| **NuttX** | The real-time operating system running on the Pixhawk flight controller. |
| **nvblox** | NVIDIA's GPU library that builds a live 3D map from depth images. |
| **NVMe** | Fast solid-state storage that plugs into an M.2 slot. Far more reliable than a microSD card. |
| **Offboard** | PX4's flight mode in which the companion computer streams position setpoints. Where all autonomy happens. |
| **ONNX** | Open Neural Network Exchange — a portable file format for trained models. |
| **Pixhawk** | The open hardware standard for drone flight controllers. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **QGroundControl** | The standard open-source ground control station application. |
| **RC** | Radio Control — the manual pilot's link to the aircraft. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **RTL** | Return To Launch — automatic flight back to the takeoff point, then land. The standard failsafe. |
| **RTX** | NVIDIA's desktop GPU line. Isaac Sim needs one. |
| **setpoint** | A commanded target — "go to this position", "hold this heading". What the Jetson sends the flight controller. |
| **SITL** | Software In The Loop — running the real flight firmware as a simulation on your PC, no hardware needed. |
| **SSD** | Solid State Drive. |
| **SSH** | Secure Shell — remote terminal login over the network. |
| **STM32H7** | The microcontroller chip inside a Pixhawk flight controller. |
| **TensorRT** | NVIDIA's library that optimizes a trained neural network to run fast on a specific GPU. |
| **TF** | The ROS coordinate-frame transform system (`tf2`). Tracks where every part and sensor sits relative to everything else. |
| **TOPS** | Tera-Operations Per Second — a measure of AI compute throughput. |
| **UART** | A simple two-wire serial connection between boards. How the Jetson talks to the flight controller. |
| **uXRCE-DDS** | The bridge that lets PX4 publish and subscribe to ROS 2 topics directly, without a MAVLink translation layer. |
| **VIO** | Visual-Inertial Odometry — estimating motion by combining camera images with IMU data. How you navigate without GPS. |
| **YOLO** | "You Only Look Once" — a widely used family of fast object-detection models. |
