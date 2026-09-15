# 08 — Mission Data Flow: Search → Select → Investigate

The reference workflow for this project:

1. Drone scans an area, finds candidate contacts, reports them to the operator
2. Operator reviews and selects one
3. Drone flies to it and gathers detailed video/data

This is a **human-in-the-loop** pattern: the aircraft does the searching, the human does the
deciding. It maps onto milestones M5 + M6 in [06](06-autonomy-roadmap.md) plus a custom
ground station.

---

## The constraint that determines the whole design: bandwidth

| Link | Realistic throughput | Range |
|---|---|---|
| SiK telemetry 915 MHz | ~57.6 kbps | km |
| WiFi (aircraft ↔ ground AP) | 10–50 Mbps | ~100 m |
| LTE/5G modem | 2–10 Mbps up | anywhere with coverage |
| Digital FPV (DJI O4 / Walksnail) | 60 Mbps — but a **closed pipe**, unusable for data | 15 km (FCC) |
| Analog 5.8 GHz VTX | not digital — ~17–20 MHz RF channel, composite video, **0 data capacity** | km |

**1080p30 H.264 is ~4 Mbps. The telemetry radio is ~0.058 Mbps — roughly 70× too slow.**

So the naive design — stream annotated video down continuously and let the operator watch —
is not physically possible over the long-range link. The architecture has to be
**metadata-first**:

> Send *contact reports* during search. Send *video* only during investigation, and only
> when the high-rate link is actually available.

This turns out to be better anyway: an operator scanning a map of 40 pins finds things faster
than a human watching 25 minutes of video.

### Why the FPV links do not rescue you

The FPV links are a trap worth naming explicitly. An **analog VTX carries no data at all** — it
is FM-modulated composite video, so bits/sec is the wrong unit; only OSD text drawn into the
signal by the FC gets through. A **digital unit like the DJI O4 has a genuine 60 Mbps**, far
more than LTE, but it is a proprietary air-unit-to-goggles pipe: no IP tunnel, no sockets, and
the video source is its own camera rather than the Jetson. Neither can carry contact reports,
ROS 2 topics, or the metadata overlay stream.

Treat the VTX as **the safety pilot's independent eyes**, valuable precisely because it does
not depend on the Jetson: if the companion computer hangs, the human still sees where the
aircraft is pointing. An analog VTX (~$30, ~5 g) covers that at VLOS ranges. Buy a digital
unit for the pilot's view if you want one, never as a data path.

---

## Phase 1 — Search

Everything happens onboard. Full-rate, full-resolution frames never leave the Jetson.

```
camera ──► TensorRT detector ──► tracker ──► confirmation ──► geolocation ──► contact report
 30 fps        per-frame          multi-frame    N-of-M       pixel→lat/lon       ~10 KB
              detections            tracks       filter        (TF chain)        ↓ downlink
```

### Track-before-report

Do **not** report every frame's detections. Associate detections into tracks across frames and
only emit a contact once a track survives N of the last M frames. This kills most false
positives and collapses 30 reports/second into one report per real object.

Deduplicate by **geolocation**, not by image: the same object seen on two survey passes must
produce one contact, not two.

### Geolocation — where the TF chain earns its keep

Turning a bounding box into a map pin:

```
pixel (u,v)
  └─ camera intrinsics ──► ray in camera frame
      └─ TF: camera_link → base_link → map ──► ray in world frame
          └─ intersect ground plane at known AGL ──► lat / lon
```

AGL comes from the downward rangefinder ([04](04-perception-sensors.md)) or GPS altitude minus
a terrain model. **Report the error estimate alongside the position** — the operator needs to
know a pin is ±5 m, not ±0.5 m.

Realistic accuracy at 20 m AGL: **2–10 m**, dominated by attitude error and the flat-ground
assumption. A 1° attitude error alone is ~0.35 m at that altitude; sloping terrain is usually
worse. This is why phase 3 must **re-acquire** the target visually rather than trusting the pin.

### The contact report

```json
{
  "contact_id": 47,
  "class": "vehicle",
  "confidence": 0.88,
  "lat": 40.712340, "lon": -74.011230,
  "geo_error_m": 4.5,
  "first_seen_utc": "2026-09-08T14:32:07.215Z",
  "track_frames": 23,
  "heading_deg": 210,
  "chip_jpeg_b64": "..."
}
```

- **Without** the image chip: ~300 bytes — effectively free, stream over SiK continuously.
- **With** a 200×200 JPEG chip: ~10 KB. Against SiK's raw 57.6 kbps that is ~1.4 s, but real
  throughput is roughly half the headline rate (TDM sharing, MAVLink framing, and normal vehicle
  telemetry competing), so budget **several seconds per chip**. Send chips on operator request,
  not automatically.

---

## Phase 2 — Operator selects

The ground station shows a map with contact pins, a sortable list, and thumbnails. The operator
clicks one, and `investigate(contact_id=47)` goes up.

QGroundControl will **not** do this natively — you need a custom UI. Two transports:

| Transport | When |
|---|---|
| ROS 2 topics over WiFi/LTE | Development, and whenever an IP link exists. Easiest by far. |
| MAVLink `TUNNEL` / custom messages over SiK | Long-range fallback. Contacts and commands survive when video does not. |

**The command goes to the Jetson, not the flight controller.** The Jetson plans the approach;
the FC still only ever sees position setpoints. The architecture from
[01](01-system-architecture.md) is unchanged — this workflow adds no new authority over the
aircraft.

---

## Phase 3 — Investigate

1. **Save survey state** so the search pattern can resume afterward
2. **Plan an approach** — standoff distance, altitude, orbit radius, sun angle
3. **Fly there** via Offboard setpoints
4. **Re-acquire visually.** The pin is ±5 m and the object may have moved. Switch from
   coordinate-based flight to visual servoing (M6) the moment the detector reacquires
5. **Orbit or hold, and stream video down**
6. **Record full resolution to NVMe** regardless of what the link is doing

---

## Annotated video downlink — yes, but do it the smart way

### Option A: burn-in
Draw boxes into the frame, then encode. Simple. But the operator cannot toggle it off, boxes
blur under compression, and you pay full encode cost on every annotated frame.

### Option B: clean video + parallel metadata ⭐
Encode clean video. Send box coordinates, track IDs, classes, and confidences as a **separate
low-rate metadata stream**, each packet tagged with the frame's presentation timestamp. The
ground station draws the overlay.

Advantages: boxes stay crisp vector graphics at any zoom, the operator can filter by class or
confidence, metadata costs about 1 kbps, and you can re-render the overlay later against
recorded video using an improved model.

**This is what production ISR systems do** — the standard is MISB KLV metadata multiplexed into
an MPEG-TS stream. You do not need the full standard, but the pattern is the right one.

**Use Option B.** The only fiddly part is timestamp alignment: tag every metadata packet with
the frame PTS and let the ground station buffer to match.

---

## ⚠ The Orin Nano has no hardware video encoder

**The Jetson Orin Nano module (P3767) omits NVENC.** NVIDIA left the multi-standard video
encoder out of this module — Orin NX and AGX Orin have it, Orin Nano does not. `nvv4l2h264enc`
simply is not present; you fall back to CPU encoding via `x264enc`.

Measured cost: **1080p30 at 4 Mbps with `speed-preset=ultrafast tune=zerolatency` consumes
200–250% CPU** — two-plus of your six cores — while those same cores are also running ROS 2,
camera capture, and the detection pipeline.

### Consequences for this workflow

1. **It reinforces metadata-first.** You genuinely cannot afford to encode video continuously
   through an entire survey. Encode only in phase 3.
2. **Stream 720p, not 1080p.** Roughly halves the CPU cost, and it is still plenty for a human
   identifying an object at standoff.
3. **Recording has the same problem** — writing compressed video to NVMe also needs an encoder.

### Ways around it, best first

| Approach | Notes |
|---|---|
| **Let the camera encode** | Many USB3/UVC cameras and the whole Luxonis OAK-D line output H.264/H.265 directly. The Jetson just muxes and forwards — near-zero CPU. **Best fix.** |
| Stream 720p, CPU-encode, phase 3 only | Works with any camera. Budget the cores. |
| Separate digital FPV air unit | DJI O4 / Walksnail carry their own encoders and km of range, but they are a closed pipe — no metadata injection, so you are back to burn-in (Option A). Good as an *independent* safety-pilot view. |
| Move to Orin NX | Has NVENC. The real answer if video becomes central; module-compatible with the same carriers. |

If this workflow becomes the primary mission, revisit the camera choice in
[04](04-perception-sensors.md): **an OAK-D with onboard encoding sidesteps the single biggest
weakness of the Orin Nano for this use case.**

---

## Which link carries Jetson → operator data?

Three distinct paths, carrying different things:

### 1. Jetson → FC → SiK 915 MHz → ground — contacts and commands

The Jetson does not own this radio. It borrows the flight controller's, wrapping data in
MAVLink `TUNNEL` messages sent over the companion UART.

- Nominal: 57.6 kbps serial, 64 kbps air data rate
- **Realistic: roughly half that** — TDM sharing, MAVLink framing overhead, and normal vehicle
  telemetry already competing for the link
- Carries: contact reports, operator commands, vehicle telemetry

*Option:* hang a **second SiK radio directly off a Jetson UART** (~$60) to dedicate a link to
companion traffic with no contention. Worth it if contact volume grows.

### 2. Jetson → WiFi → ground AP — **the digital link** ⭐

The Orin Nano dev kit **ships with WiFi installed**: an AzureWave AW-CB375NF 802.11ac card in
the M.2 Key E slot with dual PCB antennas. Nothing to buy.

- **10–50 Mbps**
- Range is the limit: tens of metres on stock PCB antennas; low hundreds with external antennas
  plus a directional antenna on the ground side
- **Use 5 GHz, not 2.4 GHz** — keeps it clear of the 2.4 GHz ExpressLRS control link
  ([01](01-system-architecture.md))

### 3. Jetson → LTE/5G modem → internet → operator — when range binds

- **2–10 Mbps uplink**, unlimited range wherever there is coverage
- Costs a USB or M.2 modem (~$80–150) plus a data plan
- **Gotcha:** carrier CGNAT blocks inbound connections — you cannot dial the drone. You need a
  rendezvous point, typically a WireGuard tunnel to a small VPS. Plan for that setup work.

### Rate budget for the digital link

| Stream | Rate |
|---|---|
| 720p30 H.264 video (phase 3 only) | ~2 Mbps |
| Metadata overlay — boxes, track IDs, confidences | ~10 kbps |
| Throttled ROS 2 status topics | ~100 kbps |
| **Total** | **~2.2 Mbps** |

You need ~2–3 Mbps and have 2–10 (LTE) or 10–50 (WiFi). **The link is not the constraint —
the missing hardware encoder is.**

⚠ **Do not run raw ROS 2 DDS over the wireless link.** Default DDS multicast discovery behaves
badly over lossy WiFi (discovery storms, dropouts). Use a Fast DDS discovery server, or bridge
at the edge with Zenoh (`zenoh-bridge-ros2dds`) or rosbridge websockets — and publish only what
the operator needs rather than exposing the whole topic graph.

---

## Still images instead of video — the low-bandwidth compromise

When the high-rate link is unavailable or marginal, replace live video with **still images**:
periodically pushed, or (better) requested by the operator on demand. This is the standard
low-bandwidth ISR answer, and it is already half-built — the contact report's image chip is
exactly this pattern, generalised.

### Transfer latency

Realistic net throughput, JPEG at moderate quality:

| Image | Size | SiK (~3 KB/s) | RFD900x (~13 KB/s) | WiFi |
|---|---|---|---|---|
| 200×200 detection crop | ~10 KB | ~3 s | **~0.8 s** | instant |
| 320×240 thumbnail | ~15 KB | ~5 s | **~1 s** | instant |
| 640×480 VGA | ~40 KB | ~13 s | **~3 s** | instant |
| 1280×720 | ~120 KB | ~40 s | ~9 s | instant |
| 1920×1080 | ~300 KB | ~100 s | ~23 s | instant |

Everything else in the round trip is negligible — the request is a few hundred bytes, pulling a
frame from a ring buffer is free, and JPEG-encoding a VGA frame is single-digit milliseconds on
CPU (unlike H.264 video, JPEG does not need the missing NVENC). **The transfer is the latency.**

Usability thresholds: <1 s feels instant; 1–5 s is fine for a deliberate "let me look at that";
5–15 s is tolerable but annoying; beyond ~15 s operators stop using the feature.

⭐ **This works comfortably on RFD900x and only marginally on SiK.** VGA in 3 s is a pleasant
interaction; VGA in 13 s is not. Another argument for that $250 upgrade.

### Design refinements

**Send crops, not frames.** The operator wants the *contact*, not the scene around it. A
200×200 crop at full sensor resolution beats a downscaled full frame of the same byte count.
Biggest single win available.

**Progressive JPEG.** Send a coarse pass first (~2 KB, under a second even on SiK), then refine.
The operator sees something immediately and detail fills in. Free, natively supported, and it
changes how the link *feels*.

**Ring-buffer the last N seconds** of frames on the Jetson, so "show me now" and "show me three
seconds ago" cost the same. Matters when something interesting flashed past the camera.

**Chunk, interleave, and allow cancellation.** A 13-second transfer must never starve setpoints
or telemetry, and the operator must be able to abort one mid-flight.

**Adapt to the measured link.** Track actual throughput and pick resolution/quality to hit a
latency target (e.g. "always under 5 s") rather than a fixed image size.

### Pull, not push

Prefer **on-request** as the primary mode; periodic push spends link budget on frames nobody
looks at. A low-rate heartbeat thumbnail (one ~5 KB image every 10 s) is useful for situational
awareness — make it configurable, default off.

**The downlinked image is a preview; the real data is the full-resolution frame on NVMe.** The
link decides what you *look at now*, never what you *capture*.

---

## Operating without cellular coverage

The LTE option assumes coverage. In remote areas, start by **questioning whether you need it at
all**: Part 107 requires VLOS, so the operator is legally standing within a few hundred metres
of the aircraft. You do not need internet — you need a **local point-to-point link**, and the
ground station laptop is right there. LTE solves "the operator is elsewhere", which is a BVLOS
problem you are not yet permitted to have.

What remains, cheapest first:

### 1. Better WiFi antennas — ~$100

Air-to-ground is the *favourable* case for WiFi: its usual range killer is obstruction and
multipath, and at altitude you have clean line of sight and Fresnel clearance. A directional
panel on the ground plus a decent omni on the aircraft takes you from tens of metres to
**several hundred metres, plausibly a couple of km**.

Ubiquiti airMAX gear is purpose-built for this but a Bullet is ~180 g — heavy here. A
high-power USB adapter plus a good ground antenna is lighter and cheaper.

⚠ 5 GHz WiFi sits adjacent to a 5.8 GHz VTX. Channel-plan carefully or drop one.

### 2. RFD900x instead of SiK — ~$250/pair ⭐ best value

Drop-in replacement running SiK firmware with MAVLink framing, 1 W TX, FHSS, antenna diversity.

- **>40 km range** (80 km demonstrated on a balloon)
- Air data rate selectable to **250 kbit/s** — 4× the SiK link
- Carries contacts, commands, telemetry at ranges far past anything you may legally fly

No video, but at 250 kbps **image chips become practical**: the 10 KB thumbnail that cost
several seconds over SiK now takes well under one. For the search-and-inspect workflow, this is
the highest-value remote-area upgrade per dollar.

### 3. Purpose-built MANET radio — $1,500–3,000+

Doodle Labs Mesh Rider is the canonical product: **80–100 Mbps, >100 km field-tested, 102 g,
5 W receive / 14 W peak**. Microhard pMDDL and Silvus StreamCaster occupy the same space. The
smaller **Nano²** (April 2026) targets small UAVs.

The price is the whole project budget for one component, and 14 W lands on top of a 40 W
avionics budget. **Right answer for a funded program, wrong answer for this build.**

### 4. Store-and-forward — free, and already designed in

The metadata-first architecture above gives you this. Mapping (goal 4) never needed live
downlink — photogrammetry is post-processed. Contact reports are 300 bytes and travel over
RFD/SiK anywhere. Only phase 3 wants live video, and video records to NVMe regardless.

**Treat live video as a convenience, not a requirement.** In remote areas accept degraded or
absent video, land, and pull the bag off the NVMe. An architecture decision, not a purchase.

### 5. Starlink — at the ground station, never on the drone

Starlink Mini is ~1 kg and 20–40 W, comprehensively outside the payload and power budget of a
2 kg quad. But at the **ground station** it makes sense when remote humans need contacts live
or you want cloud processing: the drone talks locally to the ground station, the ground station
carries the satellite backhaul.

This separates "aircraft link" from "internet backhaul" — the better architecture regardless of
where you operate.

### Recommended order

1. Nothing extra at first — stock WiFi is adequate at the VLOS ranges you are confined to
2. **RFD900x pair** when SiK becomes limiting (~$250)
3. Directional WiFi antennas if you want video further out (~$100)
4. Doodle Labs only if this becomes funded work
5. Starlink at the ground station only if remote humans need a live view

---

## Recommended link architecture

Two tiers, deliberately decoupled:

```
  ALWAYS UP   ── SiK 915 MHz ──►  telemetry, contact reports, operator commands   (~58 kbps)
  BEST EFFORT ── WiFi / LTE  ──►  video + metadata overlay, SSH, bag download     (Mbps)
```

If the video link drops, the mission continues: contacts still flow, commands still work, the
aircraft still flies, and video keeps recording to NVMe for post-flight review. The operator
loses eyes, not control.

Start with WiFi — cheap, and adequate for the VLOS ranges you are legally limited to anyway.
Add an LTE modem when range becomes the binding constraint.

---

## Failure modes

| Failure | Behavior |
|---|---|
| Video link drops mid-investigation | Keep flying, keep recording to NVMe, keep reporting on SiK. Operator notified. |
| Target not reacquired on approach | Hover at last known position, report `LOST`, **wait for the operator**. Do not autonomously hunt. |
| Operator link fully lost | Finish the current investigation, then RTL per policy. Never continue indefinitely. |
| Detector returns nothing for N seconds | Hold, do not drift. Written and tested in SITL first. |
| Jetson dies | PX4 drops Offboard → position hold → RTL. Unchanged from [01](01-system-architecture.md). |

**Human-in-the-loop is also the legal posture.** Part 107 requires VLOS and an operator able to
take control at all times ([07](07-regulatory-and-safety.md)), so "operator selects the target"
is not a limitation to engineer around — it is the required design.

## Sources

- [Software Encode in Orin Nano — NVIDIA Jetson Linux Developer Guide](https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/SD/Multimedia/SoftwareEncodeInOrinNano.html)
- [Jetson Orin Nano: real-time performance for video encoding — RidgeRun](https://www.ridgerun.com/post/jetson-orin-nano-how-to-achieve-real-time-performance-for-video-encoding)
- [Resolving missing nvv4l2h264enc on Jetson Orin Nano](https://industrialmonitordirect.com/blogs/knowledgebase/resolving-missing-nvv4l2h264enc-on-jetson-orin-nano)
- [RFD900x telemetry modem — PX4 Guide](https://docs.px4.io/main/en/telemetry/rfd900_telemetry)
- [RFDesign RFD900x specs](https://rfdesign.com.au/modems/)
- [Doodle Labs Mesh Rider radios](https://doodlelabs.com/products/)
- [Doodle Labs Nano² announcement, April 2026](https://www.unmannedsystemstechnology.com/2026/04/doodle-labs-introduces-new-nano-mesh-rider-radio/)

---

## Glossary

| Term | Meaning |
|---|---|
| **AGL** | Above Ground Level — altitude measured from the ground directly below, not from sea level. |
| **AGX** | NVIDIA's largest Jetson family (AGX Orin), a tier above Orin NX and Orin Nano. |
| **BVLOS** | Beyond Visual Line Of Sight — flying where the operator cannot see the aircraft. Needs special FAA authorization. |
| **CGNAT** | Carrier-Grade NAT — mobile networks share one public address among many users, so you cannot connect *into* a device on LTE without a relay. |
| **CPU** | Central Processing Unit — the general-purpose processor cores. |
| **DDS** | Data Distribution Service — the publish/subscribe messaging standard underneath ROS 2. |
| **ExpressLRS** | ELRS — open-source long-range radio control link for the manual pilot. |
| **FC** | Flight Controller — the real-time board (a Pixhawk here) that actually stabilizes and flies the aircraft. |
| **FCC** | Federal Communications Commission — US radio regulator. "15 km (FCC)" means range under US power limits. |
| **FHSS** | Frequency Hopping Spread Spectrum — a radio technique that constantly changes frequency to resist interference. |
| **FPV** | First Person View — flying by watching a live camera feed from the aircraft. |
| **GPS** | Global Positioning System — satellite positioning. |
| **ISR** | Intelligence, Surveillance, Reconnaissance — the industry term for observe-and-report missions. |
| **JPEG** | The common compressed still-image format. |
| **KLV** | Key-Length-Value — the metadata encoding professional video systems use to attach data to each frame. |
| **LTE** | Mobile phone data network (4G). 5G is its successor. |
| **MANET** | Mobile Ad-hoc NETwork — radios that mesh together and relay for each other. |
| **MAVLink** | The standard lightweight messaging protocol between drones, ground stations and companion computers. |
| **MISB** | Motion Imagery Standards Board — defines how metadata rides alongside video in professional systems. |
| **NVENC** | NVIDIA's hardware video encoder. **Absent on the Orin Nano** — a key constraint in this project. |
| **NVMe** | Fast solid-state storage that plugs into an M.2 slot. Far more reliable than a microSD card. |
| **NX** | Jetson Orin NX — the step up from Orin Nano. Notably, it has NVENC. |
| **Offboard** | PX4's flight mode in which the companion computer streams position setpoints. Where all autonomy happens. |
| **OSD** | On-Screen Display — text overlaid onto an FPV video feed. |
| **Part 107** | The FAA rule covering commercial/professional drone operation. Requires a remote pilot certificate and VLOS. |
| **PCB** | Printed Circuit Board. |
| **PTS** | Presentation Time Stamp — marks when a video frame should be displayed. Used to align metadata with frames. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **QGroundControl** | The standard open-source ground control station application. |
| **RF** | Radio Frequency. |
| **RFD900x** | A long-range (40 km+) telemetry radio; a drop-in upgrade from the SiK radio. |
| **ROS 2** | Robot Operating System 2 — the middleware framework robots use to pass messages between programs. Not an operating system. |
| **RTL** | Return To Launch — automatic flight back to the takeoff point, then land. The standard failsafe. |
| **SiK** | The firmware (and the radios running it) used for standard 915/433 MHz MAVLink telemetry. |
| **SITL** | Software In The Loop — running the real flight firmware as a simulation on your PC, no hardware needed. |
| **SSH** | Secure Shell — remote terminal login over the network. |
| **TDM** | Time Division Multiplexing — sharing one radio channel by taking turns, which halves effective throughput. |
| **TensorRT** | NVIDIA's library that optimizes a trained neural network to run fast on a specific GPU. |
| **TF** | The ROS coordinate-frame transform system (`tf2`). Tracks where every part and sensor sits relative to everything else. |
| **UART** | A simple two-wire serial connection between boards. How the Jetson talks to the flight controller. |
| **USB3** | USB 3.0 — fast USB, and a notorious source of 2.4 GHz radio noise. |
| **UVC** | USB Video Class — the standard that lets a USB camera work without custom drivers. |
| **VGA** | 640×480 resolution. |
| **VLOS** | Visual Line Of Sight — the operator can see the aircraft unaided. Required under FAA Part 107. |
| **VPS** | Virtual Private Server — a rented cloud machine, used here as a rendezvous point for LTE links. |
| **VTX** | Video Transmitter — the FPV video downlink to the pilot. |
| **WireGuard** | A simple modern VPN, used to reach a drone sitting behind carrier NAT. |
