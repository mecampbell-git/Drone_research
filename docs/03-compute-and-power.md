# 03 — Getting the Orin Nano Airborne

**You can fly the NVIDIA reference carrier board.** It is not purpose-built for flight, but it
is entirely workable for a research testbed — the measured **183 g** (module + carrier +
heatsink/fan + 128 GB NVMe) is well inside this airframe's payload budget.

What actually needs attention: how it is powered, how the connector is secured, how it is
mounted, and how it is cooled. None of these require different hardware.

## The module

| Spec | Value |
|---|---|
| AI performance | 67 TOPS (INT8, sparse) — 1.7× the original Orin Nano |
| GPU | 1024-core Ampere, 32 tensor cores |
| CPU | 6-core Arm Cortex-A78AE |
| Memory | 8 GB LPDDR5, shared CPU/GPU |
| Power modes | 7 W / 15 W / 25 W (MAXN "Super") |
| Dev kit price | $249 |

### ⚠ No hardware video encoder

The Orin Nano module (P3767) **omits NVENC**. Orin NX and AGX Orin have a hardware video
encoder; this one does not. Encoding falls back to CPU `x264enc`, costing **200–250% CPU
(two-plus of six cores) for 1080p30 @ 4 Mbps**. This materially shapes any video-downlink
design — see [08](08-mission-data-flow.md) for the workarounds (camera-side encoding is the
best one).

**8 GB is unified memory shared between CPU and GPU.** This is the real constraint on this
platform, not TOPS. A TensorRT detection engine, a stereo depth pipeline, cuVSLAM, and ROS 2
image transport all compete for the same 8 GB. Budget it explicitly and prefer zero-copy
(`type_adaptation` / NITROS in Isaac ROS) over naive ROS message copies.

### Upgrade path worth knowing about

NVIDIA announced the **Jetson Orin Nano 2** on 2026-08-25: 78 TOPS, ~2× the inference
performance of Orin Nano Super, **same form factor**, 40% less power at equal performance.
Module and dev kit expected **1H 2027**, pricing unannounced.

Implication: **do not wait for it**, but do design the mounting and power so the module is
swappable. If you use a standard 260-pin SO-DIMM carrier, it should be a drop-in later.

## Power — the #1 integration mistake

The dev kit ships with a 19 V wall supply, but the carrier's **DC barrel jack accepts 9–20 V**
(datasheet: 3 A max; board markings say 5 A). A 4S LiPo runs **16.8 V charged down to ~13.2 V at
landing** — entirely inside that range. So the flight battery can power the Jetson directly
through a regulator, with no special carrier hardware.

| Load | Draw |
|---|---|
| Orin Nano @ MAXN | 25 W |
| USB3 camera | 2–5 W |
| NVMe SSD | 3–5 W |
| Peripherals / margin | 5 W |
| **Total** | **~40 W** |

### Use a regulated DC-DC at ~19 V

**Not** a 5 V BEC — that advice applies to a bare module on a third-party carrier. This carrier
does its own regulation and wants 9–20 V at the input.

**Not** direct from the battery either. Nominally it is in range, but ESC switching transients
can spike past the 20 V ceiling and kill the board.

**Why 19 V and not 12 V:** the jack's current rating binds. At 12 V, 40 W draws 3.3 A — at or
over the 3 A datasheet limit. At 19 V it is **2.1 A**, comfortable.

Specify a **buck-boost DC-DC module, output ~18–19 V, rated 5 A**, fed from the battery through
the PDB — **not** off the flight controller's BEC, which is sized for servos and receivers
(2–3 A). Browning out your flight controller mid-flight is the one failure you cannot recover
from.

Add a bulk capacitor (470 µF+ low-ESR) at the converter input to absorb throttle transients.

### What happens as the battery drains

The output does **not** change. A regulated converter holds 19 V regardless of input voltage —
the Jetson never sees the pack sag. What changes is the *input current*, because the Jetson is a
roughly constant-power load:

| Battery state | Pack voltage | Input current (40 W load, ~90% eff.) |
|---|---|---|
| Fully charged | 16.8 V | 2.6 A |
| Nominal | 14.8 V | 3.0 A |
| Low-battery RTL fires | 14.0 V | 3.2 A |
| Critical / landing | 13.2 V | 3.3 A |
| Momentary sag under a hard punch | ~12.0 V | 3.7 A |

Output stays pinned at **19 V / 2.1 A** throughout — under the jack's 3 A rating at every point.

⭐ **Size the converter for the empty end, not the full end.** Worst case is ~3.7–4 A input,
which is why the spec is a 5 A part. Size wire gauge and connectors for that number too.

**With 4S it is always boosting** — 16.8 V charged is below 19 V, so a pure boost module would
do. Buck-boost is recommended anyway to future-proof: on 6S, 25.2 V charged exceeds the jack's
20 V ceiling and you would need to buck instead.

**Boost topologies pass input through when disabled** (inductor + diode path), so at power-on
the Jetson may briefly see raw pack voltage. At 4S that is 16.8 V — inside range, harmless. On
6S it would be 25.2 V and would damage the board.

The battery failsafe protects this end anyway: PX4 triggers RTL near 3.5 V/cell (14 V) and
critical at ~3.3 V/cell (13.2 V), so you land long before the converter's input range (typically
down to 8–9 V) is stressed.

**The converter earns its place on transients, not on discharge.** Steady 13.2 V is fine for the
jack directly; it is throttle-punch sag and ESC switching spikes that kill boards. The converter
decouples the Jetson from both.

Budget ~4 W dissipated in the converter itself (44 W in, 40 W out). Give it some airflow — do
not bury it in foam.

### ⚠ Secure the barrel connector

Barrel jacks vibrate loose. One unseating mid-flight means instant loss of the companion
computer. Hot-glue the connector, strain-relieve and zip-tie the cable, or solder to the jack
pads directly. This is a five-minute job that prevents a whole class of mystery failures.

### The full power budget — where the watts actually go

The 40 W above is **avionics only**. Propulsion dwarfs it.

Estimating hover power: Holybro quotes ~18 min hover for the stock ~1.6 kg aircraft on a
5000 mAh 4S pack (74 Wh). At ~80% usable capacity that implies **~200 W at hover**. Hover power
scales roughly with weight^1.5 (momentum theory), so at our 1.93 kg:

    200 W × (1.93 / 1.60)^1.5 ≈ 265 W

| System | Hover | Peak (full throttle) |
|---|---|---|
| 4× motors + ESCs | ~265 W | ~1,100 W |
| Jetson + camera + NVMe | 40 W | 40 W |
| FC, GPS, SiK radio, RC RX | ~5 W | ~5 W |
| **Total** | **~310 W** | **~1,150 W** |
| Current @ 14.8 V | **~21 A** | **~78 A** |

So the Jetson is **~13% of hover draw** — significant but not dominant. During aggressive
flight it drops to ~3%, because its draw is constant while propulsion is not.

Note the ESC headroom: 4× 20 A BLHeli_S gives 80 A total, against ~78 A at full throttle. That
is thin. This airframe is not built for punch-outs, and with payload aboard you should not be
flying it that way anyway.

### Weight costs you more than watts

Worth quantifying, because it is counterintuitive:

| Change | Flight time |
|---|---|
| Stock aircraft, 1.6 kg | 18 min |
| …+383 g payload (mass only) | **13.6 min** — costs 4.4 min |
| …+40 W avionics draw on top | **11.6 min** — costs a further 2.0 min |

**The added mass costs roughly twice what the added wattage does.** If endurance ever becomes
the binding constraint, shaving grams beats optimizing the Jetson's power mode — though
`nvpmodel` switching to the 15 W mode when you are not inferencing is scriptable and still worth
doing.

### Realistic flight time: 11–13 min

This supersedes the more optimistic figure quoted earlier in this project. And note that every
published hover time — Holybro's included — is measured hovering in still air. Wind,
maneuvering, and climbing all cost more. **Budget 60–70% of hover time for real mission
endurance**, so plan missions around 8 minutes and land with reserve.

These are scaling estimates. The real number comes from **M3**, where you fly a full pack with
everything aboard and measure it.

## Why not 30 minutes, like a consumer drone?

A DJI Mavic 3 costs $1,500–2,000 and quotes **40 min hover / 46 min flight**. This build costs
~$1,400 and gets ~12 min. The gap is real and worth understanding rather than explaining away.

### Compare like with like

Consumer figures are ideal-condition marketing: no wind, sea level, optimal speed. Real Mavic 3
owners report ~30–35 min. The honest comparison is **~32 min vs our ~12 min hover** — still a
2.5–3× gap.

### Where the gap comes from

| Factor | Mavic 3 | This build | Effect |
|---|---|---|---|
| All-up weight | 895 g | 1,930 g | **2.16× heavier** |
| Prop diameter | 9.4" | 10" | barely larger |
| **Disc loading** | **4.9 kg/m²** | **9.5 kg/m²** | **1.95× worse** |
| Battery chemistry | Li-ion, 230 Wh/kg | LiPo, 148 Wh/kg | **55% less energy per gram** |
| Battery mass fraction | 37% of AUW | 26% of AUW | less of the aircraft is fuel |

**Disc loading is the dominant term.** Induced power per unit thrust scales with the square root
of disc loading, so being 1.95× worse costs ~40% more power *for every gram lifted* — on top of
lifting 2.16× as many grams. Multiplied out: ~3× the hover power, which is exactly the gap
observed. We are carrying more than twice the weight on barely more rotor disc.

Add integration overhead (separate FC, carrier, converter, connectors, standoffs — easily
200–300 g that an injection-moulded airframe does not carry) and aerodynamics (an open carbon
plate with a box on top versus a streamlined moulded body).

### The thing the comparison hides

**The Mavic is not carrying a Jetson.** Its obstacle avoidance runs on a fixed-function vision
ASIC drawing perhaps 5–10 W. This build carries 183 g and ~40 W of *programmable* GPU compute.

That is the entire trade: **endurance bought programmability.** No consumer drone will run your
own YOLO model at any price — the compute is closed. You are not paying more for less; you are
paying endurance for a capability that is not otherwise purchasable.

### Closing the gap — two levers

**Lever 1: Li-ion instead of LiPo (~$150–250, biggest single win)**

Replace the LiPo pouch pack with one built from **cylindrical 21700 lithium-ion cells**.

Li-ion is normally rejected in drones on C-rate grounds — racers pull 100+ A instantly and
Li-ion's higher internal resistance sags. **But this aircraft hovers at ~21 A.** Our mission
profile is exactly what Li-ion is good at, which is why long-range FPV has moved to it and
racing has not.

**Configuration: 4S3P** — four in series (14.8 V, matching the 4S ESCs) × three in parallel.
Twelve cells.

| | 4S 5000 mAh LiPo | 4S3P Molicel P42A |
|---|---|---|
| Capacity | 5.0 Ah | 12.6 Ah |
| Energy | 74 Wh | **186 Wh** |
| Weight | ~500 g | ~900 g |
| Density | 148 Wh/kg | **207 Wh/kg** |
| Current capability | 100 A | 135 A (peak demand ~78 A) |

AUW 1.93 → 2.33 kg, hover ~350 W, flight time **11.6 → ~24 min**.

⚠ **Cell choice is not optional.** Use high-drain cells — Molicel P42A (4200 mAh, 45 A), P45B,
Samsung 40T, Murata VTC6. Standard energy cells (NCR18650GA and similar) deliver 8–10 A, so 3P
gives ~30 A — under peak and marginal even at hover. Wrong cells produce a pack that sags into a
brownout.

### What changes when you switch

| Change | Consequence |
|---|---|
| **PX4 battery config** | Li-ion discharges safely to 3.0 V/cell (12.0 V for 4S) vs LiPo's 3.5 V. Reset `BAT1_V_EMPTY`, `BAT1_V_CHARGED` and the internal-resistance parameter — otherwise failsafe fires ~30% early and you throw away the endurance you just bought. |
| **More sag under load** | Higher internal resistance → bigger dips on throttle punches. Prefer coulomb-counted capacity over raw voltage for the remaining-charge estimate. |
| **Poor in cold** | Significant capacity and current loss below ~10 °C; LiPo handles cold better. Keep packs warm until launch in winter. |
| **Slower charging** | 0.5–1C vs LiPo's 1–5C. A 12.6 Ah pack at 0.5C takes ~2 hours — a real hit to iteration speed on a test day. |
| **Physical fit** | Twelve 21700s is a different shape from a LiPo brick. Check it against the X500 battery tray. |

⭐ **Buy assembled, do not build.** Pack building needs spot welding — never solder directly to
cells — and is genuinely dangerous done wrong. Assembled 4S3P 21700 packs sell to the long-range
FPV market for ~$150–250.

**Why not just a bigger LiPo?** A 4S 8000 mAh LiPo is ~118 Wh at ~750 g (157 Wh/kg) → **~15.6
min**. Li-ion's 24 min wins because you are buying *density*, not just carrying more of the same.

**When:** not now. Through M0–M6 the LiPo is genuinely better — faster charging, more forgiving,
better in cold, cheaper — and flights are short test sorties anyway. Switch when M9-style survey
endurance becomes the binding constraint.

### Does Li-ion cost performance? Speed, thrust and TWR

The intuitive objection — Li-ion has less current capability, so less peak thrust, so less speed
— **does not apply to this aircraft.**

**Current is not the limit.** A 4S3P P42A pack supplies **135 A continuous**; the ESCs cap at
**80 A** (4× 20 A). The pack can deliver more than the ESCs can draw.

| Flight regime | Current @ 14.8 V |
|---|---|
| Hover | ~21 A |
| Max-tilt forward flight (45°) | ~30–37 A |
| Full-throttle vertical | ~78 A |

The Li-ion current objection is real for racing quads pulling 100+ A. It is not real here.

**The actual cost is the 400 g, via thrust-to-weight:**

| | LiPo (now) | Li-ion 4S3P |
|---|---|---|
| AUW | 1.93 kg | 2.33 kg |
| Max thrust (same motors) | ~3.8 kg | ~3.8 kg |
| **Thrust-to-weight** | **~1.97** | **~1.63** |
| Max theoretical tilt | ~59° | ~52° |
| Excess thrust for climb | 18.4 N | 14.4 N (**−22%**) |
| Top speed (est.) | ~22 m/s | ~21 m/s (−5%) |
| Endurance | 11.6 min | **~24 min** |

**TWR governs everything dynamic.** ~2.0 is the usual minimum for crisp control; 1.63 is flyable
but sluggish and noticeably worse at fighting gusts — which matters for a survey aircraft in wind.

**Top speed barely moves, counterintuitively.** Speed comes from tilt: leaning by θ converts weight
into forward force `W·tan(θ)` against drag. Lower TWR allows less tilt, but a heavier aircraft has
more weight to lean — the effects nearly cancel.

**Summary of the trade: endurance doubles, climb rate drops ~22%, top speed is unchanged.**

### Climb and descent rate

**The number you will actually experience: 3 m/s.** PX4 defaults are `MPC_Z_VEL_MAX_UP = 3.0`
and `MPC_Z_VEL_MAX_DN = 1.0` m/s. Unless changed, that is your climb rate regardless of airframe
capability. Reaching 20 m AGL takes ~7 seconds.

**Power cost** (momentum theory: hover induced velocity 6.2 m/s from the 0.203 m² disc,
propulsion efficiency ~0.44 back-calculated to match the 265 W hover figure):

| Climb rate | Propulsion | Total incl. avionics | vs hover |
|---|---|---|---|
| 0 (hover) | 265 W | 310 W | — |
| **3 m/s** (PX4 default) | ~337 W | **~382 W** | **+23%** |
| 5 m/s | ~393 W | ~438 W | +41% |
| 10 m/s | ~556 W | ~601 W | +94% |

Climbing 20 m costs ~0.7 Wh, about 1% of the usable pack. **Climb energy is irrelevant to mission
planning at survey altitudes.**

**Physical ceiling:** with TWR ~1.97 there is ~18 N of excess thrust — realistically **8–12 m/s**
before frame drag and motor thermal limits dominate. Vertical drag grows sharply above ~10 m/s
because the frame presents far more area vertically than horizontally. You would never configure
this on a survey platform; 3–5 m/s up is typical.

#### ⚠ Descent is the constrained direction — vortex ring state

Descending vertically into your own downwash causes **vortex ring state**: the rotors ingest
recirculating air, lift collapses, and adding throttle makes it worse. It is a genuine
loss-of-control mode, and it is why PX4 limits descent to 1 m/s by default while allowing 3 m/s
up.

**To descend faster, descend while moving forward** so the rotors work in clean air. Know this
before raising `MPC_Z_VEL_MAX_DN`.

**With the Li-ion pack** (2.33 kg): excess thrust falls 18.4 → 14.4 N, so max climb drops ~22% to
roughly 6–9 m/s. But at the 3 m/s actually flown the penalty is invisible — ~440 W instead of
~382 W. The lower ceiling only matters if you are pushing the aircraft, which a survey platform
is not.

### How fast can this design go?

With the current LiPo, three different answers:

| | Speed |
|---|---|
| PX4 default (`MPC_XY_VEL_MAX`) | **12 m/s** (27 mph) — what you get out of the box |
| Realistic sustained, limits raised | 15–18 m/s (34–40 mph) |
| Theoretical max tilt, drag-limited | ~22 m/s (49 mph) |

The default is a *software* limit — raise `MPC_XY_VEL_MAX` and `MPC_TILTMAX_AIR` to go faster.

**Best-range speed is much lower, ~8–12 m/s.** Forward flight beats hover efficiency up to a
point (translational lift), then drag dominates. Survey missions want that band, not top speed.

⚠ **These are estimates** resting on two soft assumptions: motor thrust (~950 g/motor at 4S with
1045 props — varies by source) and drag (CdA ≈ 0.10 m², a guess for an open frame with a Jetson
box on top). Firm them up with:

- **[eCalc xcopterCalc](https://www.ecalc.ch/xcoptercalc.php)** — enter frame, motors, props,
  battery and AUW; returns thrust, TWR, hover current, max speed, flight time. The standard tool.
  Run it **before** buying a Li-ion pack.
- **M3 measurement** — fly a full pack with everything aboard and log real current. That one
  number replaces most of the guesswork here.

**Lever 2: lower disc loading — bigger props**

A 650 mm frame with 13–15" props roughly halves disc loading and cuts hover power 25–30%. This
is a *new airframe*, not a retrofit — a second-aircraft decision.

**Both together plausibly reach 25–30 min**, i.e. consumer-competitive while carrying compute a
consumer drone does not have.

### But do not optimize this yet

**Endurance is an M9 problem, not an M0 problem.** Through M0–M6 you are flying short test
sorties and landing to change code; 8-minute missions are genuinely fine and batteries are
cheap — carry six. By the time survey endurance actually binds, you will have measured hover
current for real and will know precisely which lever to pull.

## Thermal

The Orin Nano at 25 W in a sealed 3D-printed enclosure will thermal-throttle, and a throttled
Jetson silently drops your detection rate rather than erroring. Rules:

- Keep the **active heatsink/fan** from the dev kit. Do not go passive to save 40 g.
- Mount with **clear intake and exhaust**, ideally in propwash but never relying on it —
  it must survive sitting armed on the ground in the sun.
- Log `tegrastats` alongside flight logs. Thermal throttling correlates with "the autonomy got
  weird late in the flight" and you want that visible.

## Vibration and mounting

- The module and NVMe are solid-state; the **connectors** are the vulnerability. Strain-relieve
  and hot-glue every USB, CSI ribbon, and power connector.
- Mount the Jetson on **soft standoffs / gel damping**, not rigidly to the frame. The carrier's
  100×79 mm footprint fits the X500 V2's 144×144 mm plate comfortably.
- Unused connectors (4× USB-A, RJ45, DisplayPort, 40-pin header) are harmless dead weight, but
  keep them clear of props and wiring.
- The **camera** mount is the opposite problem: it needs to be *rigid* relative to the IMU for
  VIO to work, while being isolated from motor vibration. This tension is the hard part of a
  vision drone build. Balance your props properly — it is the cheapest fix available.

## Boot media

**Boot from NVMe, not microSD.** microSD cards fail from write wear and vibration, and they will
do it 40 minutes into a field session. A 500 GB M.2 2280 NVMe is ~$45 and also gives you room to
log raw video, which you will want for offline model training.

## Weight budget (path A, X500 V2)

Confidence matters as much as the numbers: only two entries here are solid.

| Item | Mass | Confidence |
|---|---|---|
| X500 V2 frame (carbon, arms, landing gear) | 610 g | **spec** — Holybro published |
| Motors (4× 2216) + ESCs + props + PDB | ~320 g | estimate — back-derived to match quoted AUW |
| Pixhawk 6C + M10 GPS + SiK radio + RC RX | ~120 g | estimate |
| 4S 5000 mAh LiPo | ~500 g | estimate — ±50 g by brand |
| **Subtotal — stock aircraft** | **~1,550 g** | *reconciles with Holybro's ~1.6 kg quoted AUW* |
| **Jetson Orin Nano + NVIDIA carrier + heatsink/fan + 128 GB NVMe** | **183 g** | **measured** |
| DC-DC converter (19 V) + wiring + bulk capacitor | ~50 g | estimate — part not chosen |
| Camera + mount | ~80 g | estimate — part not chosen |
| Remote ID broadcast module | ~30 g | estimate — part not chosen |
| Mounting trays, standoffs, damping | ~40 g | estimate |
| **Payload subtotal** | **~383 g** | |
| **Total AUW** | **~1,930 g** | **±~150 g** |

Only the 183 g Jetson figure is measured and only the 610 g frame is a published spec —
everything else is estimated. Treat **1,930 g ± 150 g** as the honest range, and reweigh the
whole aircraft at M1 once it is assembled. Flight time and hover power both scale off this
number, so an error here propagates everywhere.

Expect roughly **11–13 min** of hover flight — see the power budget above.

### Not yet included

| Future item | Mass | When |
|---|---|---|
| Downward rangefinder (TFmini / Lightware) | +10 g | **early** — cheap, big reliability gain |
| RealSense D435i + mount | +100 g | M7, obstacle avoidance |
| Analog FPV camera + VTX | +25 g | optional, ~M6 |
| Optical flow sensor (PMW3901) | +5 g | optional, GPS-denied insurance |
| LTE modem + antenna | +40 g | only if range binds |

**Realistic fully-loaded AUW: ~2,065 g** (baseline + rangefinder + depth camera + FPV).

Swapping the LiPo for a 4S3P 21700 Li-ion pack adds ~400 g net (→ **~2,465 g**) but roughly
doubles endurance — see "Why not 30 minutes, like a consumer drone?" above.

If you later need more margin: upsize to 11–12" props with matched motors, move to 6S, or step up
to a 650 mm frame. Decide **after** M3 gives you measured hover current, not before.

## Storage — 128 GB is enough for now, with one rule

Already running from NVMe, which is the right call — microSD fails from write wear and
vibration, typically mid-session.

At 128 GB, capacity is adequate through M5 but not generous:

| What you log | Rate | 128 GB holds |
|---|---|---|
| 720p30 H.264 @ 4 Mbps | ~1.8 GB/hour | ~70 hours |
| ROS 2 bag, **compressed** images + depth + odometry | ~0.2–1 GB/min | ~15–60 flights |
| ROS 2 bag, **raw** 1080p RGB @ 30 fps | ~11 GB/min | **~11 minutes** |

⚠ **Never bag raw image topics.** Record `image_transport` compressed topics instead. One
careless `ros2 bag record -a` on a raw camera topic fills the drive mid-flight and stops logging
exactly when something interesting happens.

Offload bags after every session. Revisit capacity around M7, when depth + VIO logging for
offline algorithm work is what actually consumes space.

## Sources

- [Jetson Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
- [NVIDIA Introduces Jetson Orin Nano 2](https://www.techpowerup.com/351998/nvidia-introduces-jetson-orin-nano-2-robotics-computer)
- [Holybro X500 V2 kit specs](https://holybro.com/products/px4-development-kit-x500-v2)
- [DJI Mavic 3 Classic specs — 40 min hover](https://www.dji.com/mavic-3-classic/specs)
- [Li-ion vs LiPo energy density for drones](https://blog.uavmodel.com/fpv-long-range-li-ion-battery-guide-18650-vs-21700-pack-building/)
- [The rise of 21700 batteries in drones](https://racepow.co/blogs/news/the-rise-of-21700-batteries-how-they-re-changing-the-drone-industry)
- [Jetson Orin Nano Dev Kit power supply — NVIDIA forums](https://forums.developer.nvidia.com/t/jetson-orin-nano-dev-kit-power-supply/312715)
- [Powering the Jetson Orin Nano on a drone — NVIDIA forums](https://forums.developer.nvidia.com/t/powering-the-jetson-orin-nano-on-a-drone/349716)
- [Jetson Orin Nano Developer Kit hardware layout](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html)

---

## Glossary

| Term | Meaning |
|---|---|
| **AGL** | Above Ground Level — altitude measured from the ground directly below, not from sea level. |
| **AGX** | NVIDIA's largest Jetson family (AGX Orin), a tier above Orin NX and Orin Nano. |
| **AUW** | All-Up Weight — total mass of the aircraft ready to fly, including battery and payload. |
| **BEC** | Battery Eliminator Circuit — a converter that steps battery voltage down to 5 V for electronics. |
| **CPU** | Central Processing Unit — the general-purpose processor cores. |
| **CSI** | Camera Serial Interface (MIPI CSI) — a direct ribbon-cable camera connection. Lower latency and CPU cost than USB. |
| **cuVSLAM** | NVIDIA's GPU-accelerated visual SLAM implementation, shipped as part of Isaac ROS. |
| **DC-DC** | A converter that changes one DC voltage into another (e.g. 16 V battery down to 5 V). |
| **eCalc** | A web tool for modelling multirotor propulsion (thrust, current draw, flight time) before buying parts. |
| **ESC** | Electronic Speed Controller — the board that drives one motor from the flight controller's commands. |
| **ESR** | Equivalent Series Resistance — low-ESR capacitors absorb current spikes better. |
| **failsafe** | Pre-programmed automatic behavior when something goes wrong (link lost, battery low, companion computer silent). |
| **FC** | Flight Controller — the real-time board (a Pixhawk here) that actually stabilizes and flies the aircraft. |
| **FPV** | First Person View — flying by watching a live camera feed from the aircraft. |
| **GPS** | Global Positioning System — satellite positioning. |
| **GPU** | Graphics Processing Unit — on a Jetson, the part that runs the vision models. |
| **IMU** | Inertial Measurement Unit — accelerometers plus gyroscopes. Senses motion and orientation. |
| **INT8** | 8-bit integer — a compressed number format that makes neural networks run much faster with a small accuracy cost. |
| **Isaac ROS** | NVIDIA's set of GPU-accelerated ROS 2 packages for robot perception. |
| **LiPo** | Lithium Polymer — the rechargeable battery chemistry used in drones. "4S" means four cells in series (~14.8 V). |
| **LTE** | Mobile phone data network (4G). 5G is its successor. |
| **MAXN** | The Jetson's maximum-performance power mode. |
| **NITROS** | NVIDIA Isaac Transport for ROS — passes images between nodes on the GPU without copying through the CPU. |
| **NVENC** | NVIDIA's hardware video encoder. **Absent on the Orin Nano** — a key constraint in this project. |
| **NVMe** | Fast solid-state storage that plugs into an M.2 slot. Far more reliable than a microSD card. |
| **NX** | Jetson Orin NX — the step up from Orin Nano. Notably, it has NVENC. |
| **PDB** | Power Distribution Board — splits battery power out to the ESCs and electronics. |
| **Pixhawk** | The open hardware standard for drone flight controllers. |
| **propwash** | The turbulent airflow pushed down by the propellers. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RC** | Radio Control — the manual pilot's link to the aircraft. |
| **Remote ID** | FAA requirement (14 CFR Part 89) that a drone broadcast its identity and location while flying. |
| **RGB** | Red Green Blue — an ordinary color camera, as opposed to a depth camera. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **RTL** | Return To Launch — automatic flight back to the takeoff point, then land. The standard failsafe. |
| **SiK** | The firmware (and the radios running it) used for standard 915/433 MHz MAVLink telemetry. |
| **SO-DIMM** | The edge connector standard the Jetson module plugs into. |
| **SSD** | Solid State Drive. |
| **TensorRT** | NVIDIA's library that optimizes a trained neural network to run fast on a specific GPU. |
| **TOPS** | Tera-Operations Per Second — a measure of AI compute throughput. |
| **USB3** | USB 3.0 — fast USB, and a notorious source of 2.4 GHz radio noise. |
| **VIO** | Visual-Inertial Odometry — estimating motion by combining camera images with IMU data. How you navigate without GPS. |
| **VTX** | Video Transmitter — the FPV video downlink to the pilot. |
| **YOLO** | "You Only Look Once" — a widely used family of fast object-detection models. |
