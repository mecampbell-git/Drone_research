#!/usr/bin/env python3
"""
Fly PX4 SITL with the EdgeTX transmitter as if it were the real RC link.

The transmitter, plugged in by USB as a joystick ("Joystick (HID)" mode, classic mapping),
reports its channel outputs CH1–8 as axes 0–7. This script forwards them to PX4 at 50 Hz
as MAVLink RC_CHANNELS_OVERRIDE, which PX4 publishes as `input_rc`, the same as a CRSF
receiver. So the field parameters apply unchanged: RC_MAP_* sticks and switches, the
flight-mode slots, the arm and kill switches, and the RC-loss failsafe (unplug the USB
cable, or stop this script, and PX4 reports RC lost after COM_RC_LOSS_T).

--setup sets the RC parameters of docs/10-hardware-bringup.md §5 on the SITL instance (PX4
saves them, so once is enough): the channel map, RC_CHAN_CNT (normally set by QGC's radio
calibration; PX4 ignores RC while it is 0) and COM_RC_IN_MODE 0 (the SITL rcS defaults it
to 1, MAVLink joystick only, which ignores RC input).

Usage:
  tools/edgetx_rc.py --setup     # first time, with PX4 SITL running
  tools/edgetx_rc.py             # then just this
  tools/edgetx_rc.py --device /dev/input/js1
"""

import argparse
import os
import re
import struct
import sys
import time

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil  # noqa: E402

RATE_HZ = 50
# PX4 SITL's onboard MAVLink link: listens on 14580, sends to 14540. Binding 14540 means
# we hear PX4 first and then reply to the address it sent from.
PX4_LINK = 'udpin:127.0.0.1:14540'

JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80

SETUP_PARAMS = {
    # Channels (EdgeTX model in §5: AETR, CH5 arm, CH6 mode, CH7 return, CH8 kill)
    'RC_MAP_ROLL': 1, 'RC_MAP_PITCH': 2, 'RC_MAP_THROTTLE': 3, 'RC_MAP_YAW': 4,
    'RC_MAP_ARM_SW': 5, 'RC_MAP_FLTMODE': 6, 'RC_MAP_RETURN_SW': 7, 'RC_MAP_KILL_SW': 8,
    # 3-position switch: low = Position (2), centre = Altitude (1), high = Stabilized (8).
    # Two slots per position so the centre (slot 3/4 boundary) is unambiguous.
    'COM_FLTMODE1': 2, 'COM_FLTMODE2': 2, 'COM_FLTMODE3': 1,
    'COM_FLTMODE4': 1, 'COM_FLTMODE5': 8, 'COM_FLTMODE6': 8,
    # QGC's radio calibration sets RC_CHAN_CNT; PX4 ignores RC while it is 0. The default
    # RCn_MIN/TRIM/MAX (1000/1500/2000) match what this script sends.
    'RC_CHAN_CNT': 8, 'COM_RC_IN_MODE': 0,
    # RC loss -> Return, as in the field (also the PX4 default). The other §9 parameters
    # are left alone: the 100 m geofence would end the longer sim missions in RTL.
    'NAV_RCL_ACT': 2, 'MPC_VEL_MANUAL': 4.0,
}


def find_joystick():
    """Return /dev/input/jsN of the first input device whose name contains EdgeTX."""
    with open('/proc/bus/input/devices') as f:
        for block in f.read().split('\n\n'):
            if 'EdgeTX' in block:
                m = re.search(r'\b(js\d+)\b', block)
                if m:
                    return '/dev/input/' + m.group(1)
    return None


class Joystick:
    """Non-blocking reader for the Linux joystick API; keeps the latest axis values."""

    def __init__(self, path):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = [0] * 8

    def poll(self):
        """Drain pending events. Raises OSError if the device is gone."""
        while True:
            try:
                data = os.read(self.fd, 8 * 64)
            except BlockingIOError:
                return
            if not data:
                raise OSError('joystick closed')
            for i in range(0, len(data) - 7, 8):
                _, value, kind, number = struct.unpack('IhBB', data[i:i + 8])
                if kind & ~JS_EVENT_INIT == JS_EVENT_AXIS and number < 8:
                    self.axes[number] = value

    def channels_us(self):
        return [max(1000, min(2000, round(1500 + v * 500 / 32767))) for v in self.axes]

    def close(self):
        os.close(self.fd)


def wait_heartbeat(link):
    print('Waiting for PX4 SITL heartbeat ...', flush=True)
    while True:
        msg = link.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.get_srcComponent() == 1:
            print(f'PX4 found (system {link.target_system})', flush=True)
            return


def setup(link):
    for name, value in SETUP_PARAMS.items():
        # PX4 encodes parameters bytewise: an INT32 travels bit-cast into the float field.
        # Sent as a plain float, 1 would arrive as 1065353216 (and an RC_MAP_* that large
        # segfaults PX4 when RC arrives).
        if isinstance(value, float):
            ptype, wire = mavutil.mavlink.MAV_PARAM_TYPE_REAL32, value
        else:
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32
            wire = struct.unpack('<f', struct.pack('<i', value))[0]
        for _ in range(3):
            link.param_set_send(name, wire, ptype)
            msg = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            while msg and msg.param_id != name:
                msg = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg:
                break
        if not msg:
            sys.exit(f'No reply setting {name}')
        got = (msg.param_value if ptype == mavutil.mavlink.MAV_PARAM_TYPE_REAL32
               else struct.unpack('<i', struct.pack('<f', msg.param_value))[0])
        if abs(got - value) > 1e-3:
            sys.exit(f'{name}: PX4 reports {got}, expected {value}')
        print(f'  {name:18s} = {got:g}')
    print('Parameters set (PX4 saves them).')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--device', help='joystick device (default: the EdgeTX one)')
    parser.add_argument('--setup', action='store_true',
                        help='set the RC mapping and safety parameters first')
    args = parser.parse_args()

    link = mavutil.mavlink_connection(PX4_LINK, source_system=254)
    wait_heartbeat(link)
    if args.setup:
        setup(link)

    joystick, last_print, last_open_try = None, 0.0, 0.0
    px4_state = None
    period = 1.0 / RATE_HZ
    next_tick = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if joystick is None and now - last_open_try > 1.0:
                last_open_try = now
                path = args.device or find_joystick()
                if path:
                    try:
                        joystick = Joystick(path)
                        print(f'\nTransmitter on {path}: sending RC at {RATE_HZ} Hz', flush=True)
                    except OSError:
                        joystick = None
                if joystick is None:
                    print('\rNo EdgeTX joystick: not sending RC (PX4 sees RC lost)   ',
                          end='', flush=True)

            if joystick is not None:
                try:
                    joystick.poll()
                except OSError:
                    joystick.close()
                    joystick = None
                    print('\nTransmitter unplugged: RC stopped', flush=True)
            if joystick is not None:
                ch = joystick.channels_us()
                link.mav.rc_channels_override_send(
                    link.target_system, link.target_component, *ch, *([0] * 10))
                if now - last_print > 0.2:
                    last_print = now
                    print('\r' + ' '.join(f'{i + 1}:{c}' for i, c in enumerate(ch)) + '  ',
                          end='', flush=True)

            # Arm state and flight mode. PX4's warnings ("Arming denied ...") arrive as MAVLink
            # events, which this script can't decode: read them in QGC or PX4's console.
            while (msg := link.recv_match(type='HEARTBEAT', blocking=False)) is not None:
                if msg.get_srcComponent() != 1:
                    continue
                state = ('ARMED' if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                         else 'disarmed', mavutil.mode_string_v10(msg))
                if state != px4_state:
                    px4_state = state
                    print(f'\nPX4: {state[0]}, {state[1]}', flush=True)

            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print('\nStopped: PX4 will report RC lost.')


if __name__ == '__main__':
    main()
