#!/usr/bin/env python3
"""Animated replay of a PAMI control-loop simulation. Usage: python3 plot.py <csv>"""
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

ROBOT_R = 60  # mm, visual approximation only
WHEEL_ANGLES = np.radians([0, 120, 240])
WHEEL_COLORS = ["#ff6b6b", "#4ecdc4", "#ffd166"]
POS_COLOR, TARGET_COLOR, ERR_COLORS = "#8ecae6", "#f72585", ["#8ecae6", "#b5e48c"]

if len(sys.argv) != 2:
    sys.exit("usage: python3 plot.py <csv>")
d = np.atleast_1d(np.genfromtxt(sys.argv[1], delimiter=",", names=True))
if d.size < 2:
    sys.exit(f"error: {sys.argv[1]} needs at least 2 data rows")

t, x, y, th = d["t"], d["x"], d["y"], d["theta"]
tx, ty, tth = d["target_x"], d["target_y"], d["target_theta"]
cmds = [d["cmd0"], d["cmd1"], d["cmd2"]]
ex, ey = tx - x, ty - y
eth = np.degrees((tth - th + np.pi) % (2 * np.pi) - np.pi)

plt.style.use("dark_background")
fig = plt.figure(figsize=(14, 7.5))
fig.suptitle("PAMI control sim", fontsize=14)
gs = fig.add_gridspec(3, 2, width_ratios=[1.3, 1], hspace=0.25)
ax_map = fig.add_subplot(gs[:, 0])
ax_xy = fig.add_subplot(gs[0, 1])
ax_th = fig.add_subplot(gs[1, 1], sharex=ax_xy)
ax_cmd = fig.add_subplot(gs[2, 1], sharex=ax_xy)

# Top view: fixed limits covering start, path and targets.
pad = ROBOT_R + 0.1 * max(np.ptp(np.r_[x, tx]), np.ptp(np.r_[y, ty]), 1)
all_x, all_y = np.r_[x, tx], np.r_[y, ty]
ax_map.set_xlim(all_x.min() - pad, all_x.max() + pad)
ax_map.set_ylim(all_y.min() - pad, all_y.max() + pad)
ax_map.set_aspect("equal")
ax_map.grid(alpha=0.2)
ax_map.set_xlabel("x (mm)")
ax_map.set_ylabel("y (mm)")
(trail,) = ax_map.plot([], [], color=POS_COLOR, lw=1.5, alpha=0.8)
(target_mark,) = ax_map.plot([], [], "x", color=TARGET_COLOR, ms=12, mew=2.5)
(target_head,) = ax_map.plot([], [], color=TARGET_COLOR, lw=2)
body = ax_map.add_patch(Circle((0, 0), ROBOT_R, fc=POS_COLOR, ec="white", alpha=0.25, lw=1.5))
(heading,) = ax_map.plot([], [], color="white", lw=2)
wheels = ax_map.scatter([0, 0, 0], [0, 0, 0], s=60, c=WHEEL_COLORS, edgecolors="white", zorder=3)
info = ax_map.text(0.02, 0.98, "", transform=ax_map.transAxes, va="top", family="monospace",
                   fontsize=9, bbox=dict(fc="black", ec="gray", alpha=0.7))

# Time plots: full curve faint, the part up to now bold.
series = []
def add_series(ax, data, color, label):
    ax.plot(t, data, color=color, alpha=0.3, lw=1)
    (line,) = ax.plot([], [], color=color, lw=1.8, label=label)
    series.append((line, data))

add_series(ax_xy, ex, ERR_COLORS[0], "x err")
add_series(ax_xy, ey, ERR_COLORS[1], "y err")
add_series(ax_th, eth, TARGET_COLOR, "θ err")
for k, c in enumerate(cmds):
    add_series(ax_cmd, c, WHEEL_COLORS[k], f"cmd{k}")
for lim in (-1000, 1000):
    ax_cmd.axhline(lim, color="gray", ls="--", lw=1)
for ax, label in ((ax_xy, "pos error (mm)"), (ax_th, "heading err (°)"), (ax_cmd, "wheel cmd")):
    ax.set_ylabel(label)
    ax.grid(alpha=0.2)
    ax.legend(loc="upper right", fontsize=8, ncol=3)
ax_cmd.set_xlabel("t (s)")
ax_cmd.set_xlim(t[0], t[-1])
cursors = [ax.axvline(t[0], color="white", lw=0.8, alpha=0.6) for ax in (ax_xy, ax_th, ax_cmd)]


def update(i):
    trail.set_data(x[: i + 1], y[: i + 1])
    body.center = (x[i], y[i])
    heading.set_data([x[i], x[i] + ROBOT_R * np.cos(th[i])], [y[i], y[i] + ROBOT_R * np.sin(th[i])])
    a = th[i] + WHEEL_ANGLES
    wheels.set_offsets(np.c_[x[i] + ROBOT_R * np.cos(a), y[i] + ROBOT_R * np.sin(a)])
    target_mark.set_data([tx[i]], [ty[i]])
    target_head.set_data([tx[i], tx[i] + 1.5 * ROBOT_R * np.cos(tth[i])],
                         [ty[i], ty[i] + 1.5 * ROBOT_R * np.sin(tth[i])])
    for line, data in series:
        line.set_data(t[: i + 1], data[: i + 1])
    for c in cursors:
        c.set_xdata([t[i], t[i]])
    info.set_text(f"t   {t[i]:6.2f} s\n"
                  f"pos ({x[i]:7.1f}, {y[i]:7.1f}) mm  θ {np.degrees(th[i]):6.1f}°\n"
                  f"err ({ex[i]:7.1f}, {ey[i]:7.1f}) mm  θ {eth[i]:6.1f}°")
    return [trail, target_mark, target_head, body, heading, wheels, info, *(s[0] for s in series), *cursors]


anim = FuncAnimation(fig, update, frames=len(t), interval=1000 * (t[1] - t[0]), blit=True, repeat=False)
plt.show()
