# 02 — Build Paths Compared

Four routes to a flying autonomy testbed. Prices as of September 2026, USD, excluding tax/shipping.

---

## A. Kit + Jetson retrofit  ⭐ **recommended**

Start from a known-good, pre-tuned airframe and bolt the Jetson on.

### BOM

| Item | Price | Notes |
|---|---|---|
| X500 V2 airframe + electronics incl. power module (see "Where to buy") | $676 | 500mm carbon frame, 2216 KV920 motors, 20A BLHeli_S ESCs, 1045 props, PDB, Pixhawk 6C, M10 GPS, 915 MHz SiK. **Bundle is sold out — buy decoupled** |
| Jetson Orin Nano Super Dev Kit | *owned* | 67 TOPS, 8 GB, already running from 128 GB NVMe. Measured 183 g |
| RC TX + RX (RadioMaster TX16S + ExpressLRS RX) | ~$220 | Safety pilot control — **not optional** |
| 4S 5000 mAh LiPo ×3 + charger | ~$220 | Three packs so you can actually iterate in one session |
| DC-DC buck-boost, 19 V / 5 A | ~$25 | Carrier's jack takes 9–20 V. See [03](03-compute-and-power.md) — **not** a 5 V BEC, and not the FC's BEC |
| Global-shutter USB3 camera (e.g. Arducam / Basler-class) | ~$180 | Start RGB-only |
| Remote ID broadcast module | ~$100 | Required, see [07](07-regulatory-and-safety.md) |
| Mounting: standoffs, vibration damping, 3D-printed trays, wiring | ~$80 | |
| **Milestone-1 total** | **≈ $1,501** | Decoupled RMRC order at $675.78; ~$1,565 with the DrUAV bundle |
| *Later:* stereo depth camera | +$300–500 | Unlocks obstacle avoidance & mapping |

### Where to buy — the bundle is scarce, buy it decoupled

⚠ **As of Sept 2026 the bundled X500 V2 dev kit is broadly sold out** — Holybro direct and
NewBeeDrone ($584.99) both out of stock; DrUAV has it at **$739**, well over the $579 list.

**Buy the parts separately instead.** The bundle is one scarce SKU; its contents are four common
ones.

**Actual cart, RMRC, verified Sept 2026** — all four items from one vendor:

| Item | Price | Status |
|---|---|---|
| Pixhawk 6C, plastic case (FC + JST-GH cable set only) | $214.38 | in stock |
| **PM02D power module (2S–12S)** | **$31.07** | **required — not included with the FC** |
| X500 V2 ARF Kit | $309.15 | *only 1 left* |
| M10 GPS, 10-pin, IST8310 ×**1** | $51.27 | in stock |
| SiK Telemetry Radio V3, 915 MHz, 2 pcs | $69.91 | *only 1 left* |
| **Total** | **$675.78** | $63 under the DrUAV bundle |

⚠ **The flight controller does not include a power module.** Most bare Pixhawk 6C listings ship
only the FC and a cable set. You need a **PM02/PM02D** (~$31) to give the FC battery voltage and
current sensing — what every battery failsafe in [07](07-regulatory-and-safety.md) depends on.
The X500 PDB only *distributes* power; it does not sense it. Holybro's own "+ PM02" bundles
include it; third-party listings usually do not. **Check before ordering.**

Verify the module's current rating against the ~78 A theoretical peak (PM02-family parts are
typically 60 A continuous). Fine in practice — hover is ~21 A and the ESCs cap out near 80 A
anyway, so sustained full throttle is off the table regardless.

⚠ **A second 10-pin GPS is not a redundant GPS.** The Pixhawk 6C's GPS2 port is 6-pin, so a
second 10-pin module will not plug in. Buy one; it is a spare at best.

Plastic vs aluminum case is a coin flip — aluminum shields EMI slightly better for $10, plastic
saves ~10 g.

⚠ **Check each SKU separately.** The X500 *bundle* being sold out does not mean Holybro is out of
its parts — the Pixhawk 6C shows in stock at $184.98 while the bundle does not exist. Beware
wildly marked-up listings: pixhawk.store quoted ~$600 for a 6C combo against a $185 list.

**Where to split the order.** Holybro direct is cheaper on the FC (+PM02 for $184.98 vs $245.45
at RMRC) but that means a second international shipment. Buying everything from one US vendor is
usually worth ~$60 in avoided delay. The **ARF kit is the scarce item** — build the order around
whoever has it.

If the ARF is gone everywhere, take DrUAV's X500 V2 bundle at $739 — still not the S500, because
that $80 gap buys 172 g and the Jetson mounting plate.

### Not in the airframe order

Required before M1 flies, and easy to forget:

| Item | Price |
|---|---|
| 4S 5000 mAh LiPo × 3 + charger | ~$220 |
| RC transmitter + receiver (ExpressLRS 2.4 GHz) | ~$220 |

Consolidate with the airframe order where the vendor stocks them.

*Prices and stock verified Sept 2026 and will move. Check the ARF kit as a separate SKU from the
dev kit — it may be in stock when the bundle is not.*

#### Do not buy the S500 V2 by mistake

Holybro sells a similarly-named **S500 V2 Development Kit** at ~$659. Same propulsion, same
electronics, different — and worse — frame:

| | **X500 V2** | S500 V2 |
|---|---|---|
| Price | **$579** | $659 |
| Frame weight | **610 g** | 782 g (**+172 g**) |
| Wheelbase | 500 mm | 480 mm |
| Arms | Full carbon fiber tube, 16 mm | Nylon composite w/ CF rod |
| Plates | 2 mm carbon twill | 1.5 mm |
| **Companion computer mount** | **Included — designed for Jetson/RPi** | **None** |
| Depth camera mount | Available as accessory | No |

The mounting plate is the decisive item: it is where the 183 g Jetson and the camera physically
go. On the S500 you fabricate that yourself.

The extra 172 g also costs endurance — AUW 1,930 → 2,102 g, hover ~346 W, flight time
11.6 → **~10.2 min**. You would pay $80 more for 1.4 fewer minutes and no mounting solution.

### Payload reality check

X500 V2 frame is 610 g; ~1.6 kg AUW with a 5000 mAh 4S and no payload, quoted at **~18 min hover**.
Adding the measured 183 g Jetson plus camera, BEC and mounts is **+383 g** → ~1.93 kg AUW.
Expect **11–13 min** hover flight time — and budget 60–70% of that for real missions with wind and maneuvering. See the power budget in [03](03-compute-and-power.md). That is acceptable
for a testbed but it is the honest number — see [03](03-compute-and-power.md) for the mitigation
options (bigger props/motors, 6S, or a larger frame).

**Pros:** fastest path to actually working on autonomy, which is the point of the project.
Pre-matched motor/ESC/prop combo means no tuning archaeology. Excellent PX4 documentation.
**Cons:** payload margin is tight. Frame is bigger and less agile than a custom 7". You inherit
Holybro's component choices.

---

## B. Purpose-built integrated (ARK Jetson PAB)

ARK Electronics makes carrier boards that **combine a Pixhawk-class flight controller and a
Jetson carrier on one board**, sharing power and connected internally over Ethernet.

| Item | Price |
|---|---|
| ARK Just A Jetson carrier (Orin NX/Nano) | $695–780 |
| ARK Jetson PAB V3 + ARKV6X FC bundles | $335–475 (carrier variants) up to $1,769–2,731 (full Orin NX NDAA bundles) |
| Frame, motors, ESCs, props, GPS, RC, batteries | ~$500–700 |

**Pros:** cleanest integration — one board, one power input, Ethernet between FC and Jetson
instead of a flaky UART. Lighter and tidier than two stacked boards. NDAA-compliant and
US-made, which matters if this ever becomes commercial work.
**Cons:** costs $400–700 more than path A for the same capability at this stage, and you are
back to selecting and tuning a propulsion system yourself. The integration advantage is real
but you will not feel it until milestone 4+.

**Verdict:** the right *second* aircraft. Not the right first one. The NVIDIA reference carrier
flies acceptably — see [03](03-compute-and-power.md) — so ARK buys **integration elegance, not
capability**: one board instead of two, Ethernet to the FC instead of UART, a single power input.
None of that is needed through M6.

---

## C. Scratch build from parts

Choose frame, motors, ESCs, FC, props individually.

**Pros:** maximum learning, exact payload/endurance optimization, cheapest possible parts cost.
**Cons:** weeks of eCalc modeling, procurement, soldering, and PID tuning **before you write a
line of autonomy code** — and every crash during tuning is on a one-off airframe with no spares.
The failure modes are propulsion failure modes, which teach you about propulsion, not autonomy.

**Verdict:** you have four autonomy goals and none of them are "learn multirotor propulsion
design." Skip it. If you later want a purpose-optimized airframe, you will design a far better
one having flown path A for six months.

---

## D. Simulation first

PX4 SITL + Gazebo for flight dynamics; NVIDIA Isaac Sim for photorealistic perception.

**Cost:** $0 (Isaac Sim needs an RTX GPU on your desktop — check what you have).

**Verdict:** **not an alternative — do this in parallel with A, starting immediately.**
Every offboard control node, every state machine, every "what happens when detection returns
nothing for 3 seconds" case gets debugged in sim where a bug costs a restart instead of $600.
The same ROS 2 node binary then runs on the Jetson unchanged.

---

## Recommendation

**A + D, concurrently.**

1. Start SITL today, on hardware you already own. Build the offboard control node and the
   failsafe state machine in sim.
2. Order the X500 V2 kit. Assemble, tune, and fly it manually until it is boring.
3. Bench-integrate the Jetson (props off, propellers in another room).
4. Merge: the sim-proven node now commands a real aircraft.

This spends ~$1,450 of a $3,000 budget and keeps ~$1,500 for depth sensing, spares, and the
second airframe when you decide path B is worth it.

## Sources

- [Holybro X500 V2 PX4 Development Kit](https://holybro.com/products/px4-development-kit-x500-v2)
- [Holybro S500 V2 Development Kit — the one *not* to buy](https://holybro.com/products/s500-v2-development-kit)
- [DrUAV — X500 V2 kit (alternate retailer)](https://druav.com/en-us/products/holybro-x500-v2-px4-development-kit)
- [ARK Electronics — Just A Jetson carrier](https://arkelectron.com/product/ark-just-a-jetson/)
- [ARK Jetson PAB V3 Orin Nano 8GB NDAA Bundle](https://arkelectron.com/product/ark-jetson-pab-v3-orin-nano-8gb-ndaa-bundle/)
- [Jetson Orin Nano Super Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)

---

## Glossary

| Term | Meaning |
|---|---|
| **ARF** | Almost Ready to Fly — a kit that arrives mostly assembled. |
| **ARK** | ARK Electronics — maker of integrated flight-controller + Jetson carrier boards. |
| **ARKV6X** | ARK Electronics' Pixhawk-standard flight controller board. |
| **AUW** | All-Up Weight — total mass of the aircraft ready to fly, including battery and payload. |
| **BEC** | Battery Eliminator Circuit — a converter that steps battery voltage down to 5 V for electronics. |
| **BOM** | Bill of Materials — the parts list. |
| **DC-DC** | A converter that changes one DC voltage into another (e.g. 16 V battery down to 5 V). |
| **eCalc** | A web tool for modelling multirotor propulsion (thrust, current draw, flight time) before buying parts. |
| **ESC** | Electronic Speed Controller — the board that drives one motor from the flight controller's commands. |
| **ExpressLRS** | ELRS — open-source long-range radio control link for the manual pilot. |
| **failsafe** | Pre-programmed automatic behavior when something goes wrong (link lost, battery low, companion computer silent). |
| **FC** | Flight Controller — the real-time board (a Pixhawk here) that actually stabilizes and flies the aircraft. |
| **GPS** | Global Positioning System — satellite positioning. |
| **GPU** | Graphics Processing Unit — on a Jetson, the part that runs the vision models. |
| **LiPo** | Lithium Polymer — the rechargeable battery chemistry used in drones. "4S" means four cells in series (~14.8 V). |
| **NDAA** | US National Defense Authorization Act — "NDAA-compliant" means free of restricted foreign components. |
| **NVMe** | Fast solid-state storage that plugs into an M.2 slot. Far more reliable than a microSD card. |
| **NX** | Jetson Orin NX — the step up from Orin Nano. Notably, it has NVENC. |
| **PAB** | Pixhawk Autopilot Bus — the connector standard ARK's carrier boards use. |
| **PDB** | Power Distribution Board — splits battery power out to the ESCs and electronics. |
| **PID** | Proportional-Integral-Derivative — the control-loop math you tune so the aircraft flies steadily instead of wobbling. |
| **Pixhawk** | The open hardware standard for drone flight controllers. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RC** | Radio Control — the manual pilot's link to the aircraft. |
| **Remote ID** | FAA requirement (14 CFR Part 89) that a drone broadcast its identity and location while flying. |
| **RGB** | Red Green Blue — an ordinary color camera, as opposed to a depth camera. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **RTX** | NVIDIA's desktop GPU line. Isaac Sim needs one. |
| **SiK** | The firmware (and the radios running it) used for standard 915/433 MHz MAVLink telemetry. |
| **SITL** | Software In The Loop — running the real flight firmware as a simulation on your PC, no hardware needed. |
| **TOPS** | Tera-Operations Per Second — a measure of AI compute throughput. |
| **UART** | A simple two-wire serial connection between boards. How the Jetson talks to the flight controller. |
| **USB3** | USB 3.0 — fast USB, and a notorious source of 2.4 GHz radio noise. |
