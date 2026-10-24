# 10 — Hardware Bring-up

How to take the assembled Holybro X500 V2 from a pile of parts to its first manual flight:
milestone **M1** in [06](06-autonomy-roadmap.md). No Jetson and no camera; the aircraft is the
Pixhawk 6C, M10 GPS, SiK 915 MHz telemetry, an ExpressLRS RP3 receiver, the power module and a
4S LiPo.

Two machines are involved:

| Machine | OS | Job here |
|---|---|---|
| Desktop | Ubuntu 24.04 | Builds the PX4 firmware (`~/PX4-Autopilot`, v1.17.0) |
| Laptop | Omarchy (Arch) | QGroundControl, ExpressLRS Configurator, the field |

---

## Why the firmware needs a custom build

**Stock PX4 for the Pixhawk 6C cannot read an ELRS receiver.** The 6C's RC IN port is handled by
the IO coprocessor, which decodes PPM, SBUS and DSM only. CRSF (the ELRS protocol) needs PX4's
`crsf_rc` driver on a UART, and that driver is `default n` and not enabled in
`boards/px4/fmu-v6c/default.px4board`. So we build a variant with it switched on and plug the
RP3 into TELEM2.

Fallback with stock firmware: ExpressLRS 3.3+ can set the receiver's serial protocol to **SBUS**,
and the RP3 then goes into RC IN. That loses CRSF telemetry on the transmitter and the link
quality PX4 sees. Use it only if the custom build is a problem.

## 0. Ground rules for bench work

- **Props off** and in another room until step 11. Unplugged is not enough.
- **Antennas on before power**: SiK air and ground radios, the ELRS TX module, the RP3. A radio
  transmitting with no antenna can damage itself.
- **USB power only** wherever possible. USB powers the FC, GPS, SiK and RP3 but not the ESCs, so
  the motors cannot spin. Connect the battery only for steps 7, 8 and 10.

## 1. Laptop setup (Omarchy), once

```bash
sudo usermod -aG uucp $USER     # Arch's serial-port group is uucp, not dialout; log out and in
sudo pacman -S --needed fuse2   # the QGC AppImage needs FUSE 2
systemctl status ModemManager   # if it exists and runs, it grabs the Pixhawk/SiK serial ports:
sudo systemctl disable --now ModemManager
```

- Download the QGroundControl AppImage (stable), `chmod +x`, run it.
- If its window misbehaves under Hyprland/Wayland:
  `QT_QPA_PLATFORM=xcb ./QGroundControl-x86_64.AppImage`.
- GStreamer plugins are only needed later for camera video:
  `sudo pacman -S gst-plugins-good gst-plugins-bad gst-libav`.
- ExpressLRS Configurator (for step 5): its AppImage.

## 2. Firmware: PX4 v1.17.0 with CRSF (desktop)

Same version as SITL, so parameters and behaviour match the sim.

```bash
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-sim-tools      # installs the ARM toolchain (sudo); once
echo 'CONFIG_DRIVERS_RC_CRSF_RC=y' > boards/px4/fmu-v6c/crsf.px4board
make px4_fmu-v6c_crsf                           # → build/px4_fmu-v6c_crsf/px4_fmu-v6c_crsf.px4
```

A variant file holds only the lines that differ from `default.px4board` (compare `rover.px4board`).

Flash it either way:
- **Laptop:** copy the `.px4` across. QGC → Vehicle Setup → Firmware, plug in USB, choose PX4 →
  **Advanced → Custom firmware file**.
- **Desktop:** Pixhawk on USB, `make px4_fmu-v6c_crsf upload`.

## 3. Wiring check

| Device | 6C port | Notes |
|---|---|---|
| GPS (compass, LED, safety switch) | GPS1 | Mast arrow forward |
| SiK air radio | TELEM1 | MAVLink, `MAV_0_CONFIG` (default) |
| ELRS RP3 | **TELEM2** | RP3 TX → pin 3 (FC RX), RP3 RX → pin 2 (FC TX), 5 V → pin 1, GND → pin 6 |
| Power module (PM02) | POWER1 | |
| ESC signal | FMU PWM OUT *or* I/O PWM OUT | Note which: it decides the protocol in step 8 |

Mount the RP3's two antennas apart and at ~90° to each other, clear of the carbon plates and the
battery.

## 4. QGC basic setup (USB only)

1. **Airframe:** Quadrotor x → **Holybro X500 V2**. Apply; the board reboots.
2. **Sensors → Orientations:** autopilot and external compass `ROTATION_NONE` if their arrows
   point forward.
3. Calibrate **gyroscope**, **accelerometer** (six orientations), **level horizon** (flat surface).
4. Calibrate the **compass** outdoors or well away from metal, with everything in its final
   position. Redo it when the Jetson goes on (M3).
5. **Parameters**, then reboot:
   - `RC_CRSF_PRT_CFG` = TELEM 2
   - `RC_CRSF_TEL_EN` = 1
   - `MAV_1_CONFIG` = Disabled (if it was TELEM 2)
   - `COM_RC_IN_MODE` = 0 (RC only)

## 5. RC link

1. **ELRS versions:** TX module and RP3 on the same major version (3.x with 3.x). Set the same
   **binding phrase** on both, in ExpressLRS Configurator or the RP3's Wi-Fi hotspot (it starts
   ~60 s after power-up without a link).
2. **EdgeTX model:** CRSF on the module, packet rate 150 or 250 Hz, switch mode Hybrid,
   telemetry ratio ~1:16. Channels:

   | Ch | Function | Notes |
   |---|---|---|
   | 1–4 | AETR sticks | |
   | 5 | **Arm** (2-pos) | ELRS treats CH5 as the arm channel; sent in every packet |
   | 6 | **Flight mode** (3-pos) | Position / Altitude / Stabilized |
   | 7 | **Return** (2-pos) | |
   | 8 | **Kill** (2-pos) | Somewhere you can find without looking |

   In Hybrid mode CH6–12 are low-resolution: switches only, no sliders or knobs.
3. **QGC → Radio:** calibrate. **Flight Modes:** assign CH6, arm, kill and Return. Check that
   every switch position changes what QGC shows.
4. With CRSF telemetry on, LQ, RSSI and battery appear on the transmitter.

## 6. Telemetry radio

The Holybro SiK pair is paired at the factory. Ground radio into the laptop; QGC connects at
57600 baud. Then unplug the Pixhawk's USB and power it from the battery or a USB power bank to
prove the link works over the air.

## 7. Power (battery in, props off)

1. QGC → Power: 4 cells; voltage divider and amps-per-volt for the PM02 (from Holybro's PM02
   page).
2. Compare QGC's voltage with a multimeter on the pack; adjust the divider until they match.

## 8. Motors (battery in, props off)

1. QGC → **Actuators:** assign Motor 1–4 to the outputs actually wired.
   - **FMU PWM OUT** → **DShot600**, no ESC calibration.
   - **I/O PWM OUT** → PWM 400 Hz, then ESC calibration (Power tab).
2. Actuator test sliders, one motor at a time. Check **position**:

   | Motor | Position | Spin |
   |---|---|---|
   | 1 | front-right | CCW |
   | 2 | rear-left | CCW |
   | 3 | front-left | CW |
   | 4 | rear-right | CW |

3. Check **direction** (tape on the bell makes it visible). Reverse a motor by swapping any two
   of its three wires, or with DShot's reverse option.

## 9. Safety parameters

The layers in [07](07-regulatory-and-safety.md), as parameters:

| Parameter | Value | Why |
|---|---|---|
| `NAV_RCL_ACT` | 2 (Return) | RC loss → RTL |
| `COM_RC_LOSS_T` | 0.5 s (default) | Time without RC before it counts as lost |
| `NAV_DLL_ACT` | 0 | Telemetry loss does nothing; don't couple it to RC loss |
| `COM_LOW_BAT_ACT` | 3 | Return at critical, land at emergency |
| `BAT_LOW_THR` / `BAT_CRIT_THR` / `BAT_EMERGEN_THR` | 0.25 / 0.15 / 0.07 | Conservative while learning |
| `GF_ACTION` | 3 (Return) | Geofence breach → RTL |
| `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` | 100 m / 40 m | Tight fence for early flights |
| `RTL_RETURN_ALT` | 15–20 m | Above the trees at the field |
| `MPC_VEL_MANUAL` | ~4 m/s (default 10) | Gentler Position mode while learning |

The SITL-only `NAV_DLL_ACT` fix in `minimal.launch.py` does not apply here: the real aircraft
uses airframe 4019, not the sim airframes.

## 10. Bench tests (props off; battery in for arming)

- [ ] Arm with the switch: motors idle. Disarm: they stop.
- [ ] Arm and leave throttle at the bottom: it disarms itself after a few seconds.
- [ ] **Kill switch** stops the motors immediately.
- [ ] Each mode-switch position shows the right mode. Position mode is refused without a GPS fix,
      which is expected indoors.
- [ ] **RC loss:** armed, props off, switch the transmitter off. QGC reports RC lost and the
      failsafe engages. Repeat disarmed and check the warning.
- [ ] Tilt the frame by hand: horizon and heading in QGC move the right way.
- [ ] Tug every connector; no wire can reach a prop.

## 11. First flight

**Before going outdoors:** FAA registration (the aircraft is over 250 g), TRUST or Part 107, and
Remote ID outside a FRIA. See [07](07-regulatory-and-safety.md) §1–2.

1. **Props:** CW props on CW motors (the leading edge leads in the spin direction), tightened.
2. **Conditions:** open field, nobody downwind, light wind. GPS 3D fix and "Ready to fly" in QGC.
3. **Flight 1, Position mode:** arm, throttle up smoothly to 2–3 m, hover 30–60 s with small
   inputs, land, disarm. Short and boring is the goal.
4. **Log:** pull the `.ulg` (QGC → Analyze Tools → Log Download) and run
   `tools/plot_flight.py <file>.ulg`, or upload it to review.px4.io. Look at vibration and
   accelerometer clipping, EKF innovations, and whether hover throttle settled sensibly.
5. **Rest of M1:** Altitude mode, then Stabilized; ~10 packs; the RC-loss RTL test at altitude
   (transmitter off, watch it come home, transmitter on, take control); optionally PX4 autotune.

## Practising in SITL first

EdgeTX can act as a USB joystick. Plug the transmitter into the desktop and fly the Gazebo SITL
through QGC's joystick support, with the same sticks and switches you will use in the field.
SITL needs `COM_RC_IN_MODE` set to allow a joystick; the real aircraft stays on 0.

---

## Glossary

| Term | Meaning |
|---|---|
| **AETR** | Aileron, Elevator, Throttle, Rudder: the channel order roll, pitch, throttle, yaw. |
| **CRSF** | Crossfire serial protocol: the two-way link between an ELRS receiver and the flight controller, carrying stick channels in and telemetry out. |
| **DShot** | A digital ESC protocol. No calibration needed, unlike analogue PWM. |
| **EdgeTX** | Open-source firmware for RC transmitters such as the RadioMaster TX16S. |
| **ESC** | Electronic Speed Controller: drives one brushless motor from the flight controller's signal. |
| **ExpressLRS** | ELRS: open-source long-range radio control link for the manual pilot. |
| **FC** | Flight Controller: the Pixhawk. |
| **FRIA** | FAA-Recognized Identification Area: where a drone may fly without broadcasting Remote ID. |
| **LQ** | Link Quality: the percentage of RC packets received. |
| **QGC** | QGroundControl, the PX4 ground station. |
| **RTL** | Return To Launch: automatic flight back to the takeoff point, then land. |
| **SiK** | The open firmware on the 915 MHz telemetry radios that carry MAVLink to the laptop. |
| **SITL** | Software In The Loop: the real PX4 firmware running as a simulation on the PC. |
| **UART** | A simple two-wire serial connection; TELEM1/2 and GPS1 are UARTs. |
