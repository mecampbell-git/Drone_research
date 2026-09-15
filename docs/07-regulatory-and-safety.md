# 07 — Regulatory & Safety

*Assumes US operation. Not legal advice — verify against faa.gov before flying.*

## The rules that actually constrain the design

### 1. Registration — you are over the threshold

At ~2 kg AUW this is well past the 250 g exemption. The aircraft must be **registered with the
FAA** regardless of whether you fly recreationally or under Part 107.

### 2. Remote ID — affects your BOM

14 CFR Part 89 requires drones to **broadcast identity and location in flight**. A home-integrated
build (a kit airframe with your own payload) is not a standard-Remote-ID-equipped production
aircraft, so plan on a **broadcast module** (~$100, ~30 g). Budget it now, not at the field.

### 3. Visual line of sight — this is the big one for autonomy

Under **Part 107**, a remote pilot in command must maintain visual line of sight and **be able to
take manual control at all times**. Autonomous flight is entirely legal; *unattended* or
*beyond-line-of-sight* flight is not, absent a waiver.

**Design consequence, and it is a good one:** the safety-pilot-with-override architecture in
[01](01-system-architecture.md) is not just prudent engineering, it is the legal operating model.
Build for it from day one — an RC link and a mode switch that unconditionally beats the companion
computer.

### 4. BVLOS / Part 108 — do not plan around it

The FAA published the **Part 108 NPRM on 2025-08-07**; the comment period closed 2025-10-06 with
3,000+ responses. **As of late August 2026 it is still not a final rule** — it sits at OIRA,
roughly six months past its original February 2026 target. No operator can claim Part 108
authority yet.

Treat BVLOS as a future capability, not a project assumption. Everything in the roadmap works
fine within VLOS.

### 5. Recreational vs. Part 107

- **Recreational:** pass the free **TRUST** test, fly within a community-based organization's
  safety guidelines, strictly for fun.
- **Part 107:** the remote pilot certificate. Required the moment the flight furthers *any*
  business or professional purpose — and "research that might become something" sits in a gray
  area that the certificate simply removes.

**Recommendation: get the Part 107.** It is a ~$175 knowledge test, the study material is
genuinely useful (airspace, weather, aeronautical decision-making), and it makes the airspace and
waiver questions unambiguous. Do it during the M1 assembly/tuning phase — you will have the time.

### 6. Airspace

Class G below 400 ft AGL needs no authorization. Controlled airspace needs **LAANC** authorization
(instant, via a phone app). Check every site before flying; "it's a field near my house" is how
people discover they are under a Class D shelf.

## Safety architecture

Layered, from software to physical:

| Layer | Mechanism |
|---|---|
| 1. Software | Companion sends bad/no setpoints → PX4 drops Offboard after 0.5 s → position hold |
| 2. Geofence | PX4 geofence: hard radius + altitude ceiling → RTL on breach |
| 3. Battery | Staged failsafe: warn → RTL → land |
| 4. Link loss | RC loss → RTL. Telemetry loss → continue mission (do not couple these) |
| 5. **Human** | **Safety pilot, mode switch, always in reach** |
| 6. Physical | Prop guards during M6–M7 testing; open field; nobody downrange |

**The ordering matters.** Layer 5 must work even if 1–4 are broken, which means the RC receiver
talks to the flight controller directly and the Jetson is nowhere in that path.

## Test discipline

- **Props off** for all bench work. Physically separated from the workspace, not just unplugged.
- **First autonomous flight of any new behavior:** open field, no people, low altitude, tight
  geofence, fresh battery, hand on the switch.
- **LiPo handling:** charge in a bag, on a non-flammable surface, never unattended. Retire
  puffed packs. A 5000 mAh 4S holds real energy.
- **Preflight checklist, written down, every flight.** Props tight and correct rotation, battery
  secure, GPS lock count, IMU calibrated, compass healthy, failsafes armed, Jetson booted and
  cooled, camera lens clean. Write it once; use it forever.

## Sources

- [Part 108 status, 2026](https://droneauthority.org/laws/part-108)
- [Part 108 explained — Pilot Institute](https://pilotinstitute.com/part-108-explained/)
- [New FAA rules coming in 2026](https://dronetrust.com/blogs/articles/new-faa-drone-rules-2026)
- [Part 107 vs. BVLOS](https://www.uav.org/part-107-vs-bvlos-commercial-drone-operators/)

---

## Glossary

| Term | Meaning |
|---|---|
| **AGL** | Above Ground Level — altitude measured from the ground directly below, not from sea level. |
| **AUW** | All-Up Weight — total mass of the aircraft ready to fly, including battery and payload. |
| **BOM** | Bill of Materials — the parts list. |
| **BVLOS** | Beyond Visual Line Of Sight — flying where the operator cannot see the aircraft. Needs special FAA authorization. |
| **CFR** | Code of Federal Regulations — US federal rules. "14 CFR Part 107" is the commercial drone rule. |
| **FAA** | Federal Aviation Administration — the US aviation regulator. |
| **failsafe** | Pre-programmed automatic behavior when something goes wrong (link lost, battery low, companion computer silent). |
| **geofence** | A virtual boundary. Crossing it triggers an automatic response such as RTL. |
| **GPS** | Global Positioning System — satellite positioning. |
| **IMU** | Inertial Measurement Unit — accelerometers plus gyroscopes. Senses motion and orientation. |
| **LAANC** | Low Altitude Authorization and Notification Capability — near-instant FAA airspace approval through a phone app. |
| **LiPo** | Lithium Polymer — the rechargeable battery chemistry used in drones. "4S" means four cells in series (~14.8 V). |
| **NPRM** | Notice of Proposed Rulemaking — a draft regulation published for public comment. |
| **Offboard** | PX4's flight mode in which the companion computer streams position setpoints. Where all autonomy happens. |
| **OIRA** | Office of Information and Regulatory Affairs — reviews US regulations before they become final. |
| **Part 107** | The FAA rule covering commercial/professional drone operation. Requires a remote pilot certificate and VLOS. |
| **Part 108** | The FAA's proposed BVLOS rule. Still not final as of late 2026. |
| **PX4** | Open-source flight control firmware. The one recommended in this project. |
| **RC** | Radio Control — the manual pilot's link to the aircraft. |
| **Remote ID** | FAA requirement (14 CFR Part 89) that a drone broadcast its identity and location while flying. |
| **RTL** | Return To Launch — automatic flight back to the takeoff point, then land. The standard failsafe. |
| **TRUST** | The Recreational UAS Safety Test — a free FAA test required of recreational flyers. |
| **VLOS** | Visual Line Of Sight — the operator can see the aircraft unaided. Required under FAA Part 107. |
