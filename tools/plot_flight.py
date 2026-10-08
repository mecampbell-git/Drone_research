#!/usr/bin/env python3
"""
Plot a PX4 SITL flight from its ULog.

  1. XY track: estimated position, the target PX4 was given, and the mission waypoints
  2. Velocity vs time: horizontal speed and vertical speed
  3. Heading vs time: estimated heading and heading setpoint

The target is the Goto setpoint for custom modes (px4_missions, waypoint_mission.py) or
the trajectory setpoint for offboard nodes. Background bands show PX4's flight mode.
Time is seconds since arming.

Usage:
  tools/plot_flight.py                         # newest log, mission_box.csv, show windows
  tools/plot_flight.py --mission mission1.csv  # bare name: ws/src/px4_offboard/missions/
  tools/plot_flight.py path/to/log.ulg --save plots/
"""

import argparse
import csv
import glob
import os

import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/rootfs/log')
MISSION_DIR = os.path.join(REPO, 'ws/src/px4_offboard/missions')

# Categorical colours, fixed order (dataviz reference palette, light mode)
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
INK, MUTED, GRID = '#1a1a19', '#6b6a63', '#e4e3dc'
BAND = '#f2f1ec'

NAV_STATES = {0: 'Manual', 2: 'Position', 3: 'Mission', 4: 'Hold', 5: 'Return', 14: 'Offboard',
              17: 'Takeoff', 18: 'Land'}


def newest_log():
    logs = glob.glob(os.path.join(LOG_DIR, '*', '*.ulg'))
    if not logs:
        raise SystemExit(f'No logs in {LOG_DIR}')
    return max(logs, key=os.path.getmtime)


def load_mission(name):
    path = name if os.path.dirname(name) else os.path.join(MISSION_DIR, name)
    with open(path, newline='', encoding='utf-8') as f:
        rows = [[float(c) for c in row] for row in csv.reader(f) if row]
    return path, np.array(rows)


def topic(ulog, name):
    matches = [d for d in ulog.data_list if d.name == name and d.multi_id == 0]
    return matches[0].data if matches else None


def mode_name(state):
    return NAV_STATES.get(state, 'Custom mode' if state >= 23 else f'nav_state {state}')


def mode_spans(status, t0):
    """(start_s, end_s, name) for each run of the same nav_state while armed."""
    t = (status['timestamp'] - t0) / 1e6
    armed = status['arming_state'] == 2
    spans = []
    for i in range(len(t)):
        if not armed[i]:
            continue
        state = int(status['nav_state'][i])
        if spans and spans[-1][2] == state and spans[-1][1] >= t[i - 1] - 1e-6:
            spans[-1][1] = t[i]
        else:
            spans.append([t[i], t[i], state])
    # Drop blips (e.g. the instant an executor arms in its own mode before Takeoff)
    return [(a, b, mode_name(s)) for a, b, s in spans if b - a >= 0.5]


def shade_modes(ax, spans):
    for k, (a, b, name) in enumerate(spans):
        if k % 2 == 0:
            ax.axvspan(a, b, color=BAND, zorder=0, lw=0)
        ax.text((a + b) / 2, 1.0, name, transform=ax.get_xaxis_transform(), ha='center',
                va='bottom', fontsize=8, color=MUTED, clip_on=False)


def break_wraps(t, deg):
    """Insert NaN where an angle wraps past +-180 so the line does not jump across."""
    jumps = np.nonzero(np.abs(np.diff(deg)) > 180)[0] + 1
    return np.insert(t, jumps, np.nan), np.insert(deg, jumps, np.nan)


def style(ax, title, xlabel, ylabel):
    ax.set_title(title, loc='left', color=INK, fontsize=11, pad=16)
    ax.set_xlabel(xlabel, color=MUTED)
    ax.set_ylabel(ylabel, color=MUTED)
    ax.grid(color=GRID, lw=0.8)
    ax.tick_params(colors=MUTED)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', nargs='?', help='ULog file (default: newest SITL log)')
    parser.add_argument('--mission', default='mission_box100.csv',
                        help='mission CSV whose waypoints to draw (default mission_box.csv)')
    parser.add_argument('--save', metavar='DIR', help='write PNGs to DIR instead of showing')
    args = parser.parse_args()

    log = args.log or newest_log()
    ulog = ULog(log)
    lp = topic(ulog, 'vehicle_local_position')
    status = topic(ulog, 'vehicle_status')
    armed_at = status['timestamp'][np.argmax(status['arming_state'] == 2)]
    t = (lp['timestamp'] - armed_at) / 1e6
    spans = mode_spans(status, armed_at)
    in_flight = (t >= 0) & (t <= (spans[-1][1] if spans else t[-1]))

    # Target: Goto setpoint if a custom mode flew, else the offboard trajectory setpoint
    goto = topic(ulog, 'goto_setpoint')
    if goto is not None:
        tgt_t, tgt = (goto['timestamp'] - armed_at) / 1e6, goto
        tgt_heading = np.where(goto['flag_control_heading'] > 0, goto['heading'], np.nan)
        tgt_label = 'Goto target'
    else:
        tgt = topic(ulog, 'trajectory_setpoint')
        tgt_t = (tgt['timestamp'] - armed_at) / 1e6
        tgt_heading = tgt['yaw']
        tgt_label = 'Trajectory setpoint'
    tgt_in = (tgt_t >= 0) & (tgt_t <= t[in_flight][-1])

    mission_path, wp = load_mission(args.mission)
    name = os.path.basename(log)
    print(f'Log: {log}\nMission: {mission_path}\nModes: '
          + ', '.join(f'{n} {a:.0f}-{b:.0f} s' for a, b, n in spans))

    figs = {}

    # 1. XY track. NED: x = north, y = east, so plot east horizontally and north up.
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(lp['y'][in_flight], lp['x'][in_flight], color=BLUE, lw=2, label='Vehicle')
    ax.plot(tgt['position[1]'][tgt_in], tgt['position[0]'][tgt_in], color=ORANGE, lw=1.2,
            ls=(0, (4, 3)), label=tgt_label)
    stop = (wp[:, 3] > 0) | (np.arange(len(wp)) == len(wp) - 1)
    # Stops drawn last so they stay visible where a fly-through waypoint shares the spot
    ax.scatter(wp[~stop, 1], wp[~stop, 0], s=70, marker='o', facecolor='white',
               edgecolor=AQUA, linewidth=2, zorder=3, label='Waypoint (fly-through)')
    ax.scatter(wp[stop, 1], wp[stop, 0], s=70, marker='s', color=AQUA, edgecolor='white',
               linewidth=2, zorder=4, label='Waypoint (stop)')
    labels = {}     # one label per position, listing every waypoint index there
    for i, (n, e, d, delay) in enumerate(wp):
        labels.setdefault((round(e, 1), round(n, 1)), []).append(str(i))
    for (e, n), idx in labels.items():
        ax.annotate(', '.join(idx), (e, n), xytext=(7, 7), textcoords='offset points',
                    color=INK, fontsize=9)
    ax.set_aspect('equal', adjustable='datalim')
    style(ax, f'XY track — {name}, {os.path.basename(mission_path)}', 'East [m]', 'North [m]')
    ax.legend(frameon=False, loc='best', labelcolor=INK)
    figs['xy'] = fig

    # 2. Velocity vs time
    fig, ax = plt.subplots(figsize=(10, 4.5))
    shade_modes(ax, spans)
    ax.plot(t[in_flight], np.hypot(lp['vx'], lp['vy'])[in_flight], color=BLUE, lw=2,
            label='Horizontal speed')
    ax.plot(t[in_flight], -lp['vz'][in_flight], color=ORANGE, lw=2,
            label='Vertical speed (up +)')
    ax.axhline(0, color=GRID, lw=1)
    style(ax, f'Velocity — {name}', 'Time since arming [s]', 'Speed [m/s]')
    ax.legend(frameon=False, loc='upper right', labelcolor=INK)
    figs['velocity'] = fig

    # 3. Heading vs time (degrees, 0 = north, 90 = east)
    fig, ax = plt.subplots(figsize=(10, 4.5))
    shade_modes(ax, spans)
    ax.plot(*break_wraps(t[in_flight], np.degrees(lp['heading'][in_flight])), color=BLUE,
            lw=2, label='Heading (estimated)')
    ax.plot(*break_wraps(tgt_t[tgt_in], np.degrees(tgt_heading[tgt_in])), color=ORANGE,
            lw=1.2, ls=(0, (4, 3)), label='Heading setpoint')
    ax.set_ylim(-185, 185)
    ax.set_yticks(range(-180, 181, 90))
    style(ax, f'Heading — {name}', 'Time since arming [s]', 'Heading [deg, 0 = north]')
    ax.legend(frameon=False, loc='upper right', labelcolor=INK)
    figs['heading'] = fig

    for fig in figs.values():
        fig.tight_layout()
    if args.save:
        os.makedirs(args.save, exist_ok=True)
        stem = os.path.splitext(name)[0]
        for key, fig in figs.items():
            path = os.path.join(args.save, f'{stem}_{key}.png')
            fig.savefig(path, dpi=130)
            print(f'Wrote {path}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
