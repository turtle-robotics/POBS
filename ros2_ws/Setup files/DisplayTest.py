"""
Kalman Filter Test Results Visualiser
======================================
Reads generatedTest_results.csv produced by testFromFile() and plots:
  1. Estimated 3-D trajectory
  2. Position components over time  (x, y, z separate)
  3. Orientation quaternion over time
  4. Sensor-update breakdown (step counts by type)
  5. Accumulated position error vs true state
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401

STEP_MS = 0.01   # each step represents 0.01 ms

# ── Locate the results file relative to this script ──────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
RESULTS_FILE = os.path.join(SCRIPT_DIR, "generatedTest_results.csv")
TRUE_FILE    = os.path.join(SCRIPT_DIR, "trueState.csv")

if not os.path.exists(RESULTS_FILE):
    print(f"ERROR: cannot find {RESULTS_FILE}")
    print("Run the C++ test first to generate the file.")
    sys.exit(1)

# ── Load data ─────────────────────────────────────────────────────────────────
df = pd.read_csv(RESULTS_FILE)

# ── Step limit prompt ─────────────────────────────────────────────────────────
total_steps = int(df["step"].max())
print(f"Data contains {total_steps} steps ({total_steps * STEP_MS:.3f} ms).")
# raw = input(f"How many steps to display? [default: all / {total_steps}]: ").strip()
raw=100000
if raw == "":
    max_steps = total_steps
else:
    try:
        max_steps = int(raw)
    except ValueError:
        print(f"Invalid input '{raw}', showing all steps.")
        max_steps = total_steps
max_steps = min(max(max_steps, 1), total_steps)
print(f"Displaying {max_steps} steps ({max_steps * STEP_MS:.3f} ms).")

df    = df[df["step"] <= max_steps].copy()
imu   = df[df["sensor"] == "IMU"]
depth = df[df["sensor"] == "DEPTH"]
gps   = df[df["sensor"] == "GPS"]

true_df = pd.read_csv(TRUE_FILE) if os.path.exists(TRUE_FILE) else None
if true_df is None:
    print("WARNING: trueState.csv not found — true trajectory will not be shown.")

# Convert step numbers to milliseconds for every estimated series
df["time_ms"] = df["step"] * STEP_MS

# Scale true-state time axis so its full duration maps to the full simulation
# duration (total_steps * STEP_MS), then trim to the chosen step limit.
if true_df is not None:
    true_time_scale = (total_steps * STEP_MS) / true_df["t"].max()
    true_df["time_ms"] = true_df["t"] * true_time_scale
    true_df = true_df[true_df["time_ms"] <= max_steps * STEP_MS].copy()
else:
    true_time_scale = 1.0

# ── Figure layout ─────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 14))
fig.suptitle("Kalman Filter Test Results", fontsize=15, fontweight="bold")
gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.50, wspace=0.35)

ax3d  = fig.add_subplot(gs[0, 0], projection="3d")
ax_xy = fig.add_subplot(gs[0, 1])
ax_x  = fig.add_subplot(gs[1, 0])
ax_y  = fig.add_subplot(gs[1, 1])
ax_z  = fig.add_subplot(gs[2, 0])
ax_q  = fig.add_subplot(gs[2, 1])
ax_v  = fig.add_subplot(gs[3, :])

# ── 1. 3-D trajectory ─────────────────────────────────────────────────────────
if true_df is not None:
    ax3d.plot(true_df["x"], true_df["y"], true_df["z"],
              lw=1.2, color="gray", ls="--", label="True path", zorder=2)
ax3d.plot(df["pos_x"], df["pos_y"], df["pos_z"],
          lw=1.2, color="steelblue", label="Estimated path", zorder=3)
ax3d.scatter(*df[["pos_x", "pos_y", "pos_z"]].iloc[0],  color="green",  s=50, zorder=5, label="Start")
ax3d.scatter(*df[["pos_x", "pos_y", "pos_z"]].iloc[-1], color="red",    s=50, zorder=5, label="End")

if not gps.empty:
    ax3d.scatter(gps["pos_x"], gps["pos_y"], gps["pos_z"],
                 color="orange", s=20, zorder=4, label="GPS update")

ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")
ax3d.set_title("3-D Estimated Trajectory")
ax3d.legend(fontsize=7)

# ── 2. X-Y top-down view ──────────────────────────────────────────────────────
if true_df is not None:
    ax_xy.plot(true_df["x"], true_df["y"], lw=1, color="gray", ls="--", label="True path")
ax_xy.plot(df["pos_x"], df["pos_y"], lw=1, color="steelblue", label="Estimated")
if not gps.empty:
    ax_xy.scatter(gps["pos_x"], gps["pos_y"], color="orange", s=15, zorder=4, label="GPS update")
ax_xy.set_xlabel("X (m)"); ax_xy.set_ylabel("Y (m)")
ax_xy.set_title("Top-Down View (X-Y)")
ax_xy.legend(fontsize=7); ax_xy.grid(True, alpha=0.3)

# ── 3. Position components over time ─────────────────────────────────────────
def _mark_sensors(ax):
    """Add faint vertical lines where GPS measurements occur (in ms)."""
    for s in gps["step"]:
        ax.axvline(s * STEP_MS, color="orange", alpha=0.15, lw=0.6)

for ax, col, true_col, color, title in [
    (ax_x, "pos_x", "x", "royalblue",  "Position X over time"),
    (ax_y, "pos_y", "y", "seagreen",   "Position Y over time"),
    (ax_z, "pos_z", "z", "firebrick",  "Position Z (depth) over time"),
]:
    if true_df is not None:
        ax.plot(true_df["time_ms"], true_df[true_col],
                lw=1, color="gray", ls="--", label="True")
    ax.plot(df["time_ms"], df[col], lw=1, color=color, label="Estimated")
    if not gps.empty and col in ("pos_x", "pos_y"):
        ax.scatter(gps["step"] * STEP_MS, gps[col], color="orange", s=20, zorder=4, label="GPS update")
    _mark_sensors(ax)
    ax.set_xlabel("Time (ms)"); ax.set_ylabel("m")
    ax.set_title(title); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

# ── 4. Quaternion orientation ─────────────────────────────────────────────────
for comp, col in [("l (w)", "ori_l"), ("i", "ori_i"), ("j", "ori_j"), ("k", "ori_k")]:
    ax_q.plot(df["time_ms"], df[col], lw=0.9, label=comp)
ax_q.axhline(0, color="black", lw=0.5, ls="--")
ax_q.set_xlabel("Time (ms)"); ax_q.set_ylabel("Component value")
ax_q.set_title("Orientation Quaternion over time")
ax_q.legend(fontsize=7); ax_q.grid(True, alpha=0.3)

# ── 5. Estimated velocity ─────────────────────────────────────────────────────
for comp, est_col, true_col, color in [("vx", "vel_a", "vx", "royalblue"),
                                        ("vy", "vel_b", "vy", "seagreen"),
                                        ("vz", "vel_c", "vz", "firebrick")]:
    if true_df is not None:
        ax_v.plot(true_df["time_ms"], true_df[true_col],
                  lw=1, color=color, ls="--", alpha=0.5)
    ax_v.plot(df["time_ms"], df[est_col], lw=1, label=comp, color=color)
_mark_sensors(ax_v)
ax_v.set_xlabel("Time (ms)"); ax_v.set_ylabel("m/s")
ax_v.set_title("Estimated Velocity over time — dashed = true")
ax_v.legend(fontsize=8); ax_v.grid(True, alpha=0.3)

# ── Sensor update counts (text box) ──────────────────────────────────────────
counts = df["sensor"].value_counts()
info = "\n".join(f"{k}: {v}" for k, v in counts.items())
fig.text(0.01, 0.01, f"Sensor updates:\n{info}",
         fontsize=8, verticalalignment="bottom",
         bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.4))

plt.savefig(os.path.join(SCRIPT_DIR, "kalman_results.png"), dpi=150, bbox_inches="tight")
print(f"Saved kalman_results.png")

# ── Accumulated error magnitude figure ───────────────────────────────────────
if true_df is not None:
    time_ms_vals = df["time_ms"].values
    t_norm_ms    = true_df["time_ms"].values

    true_x_interp = np.interp(time_ms_vals, t_norm_ms, true_df["x"].values)
    true_y_interp = np.interp(time_ms_vals, t_norm_ms, true_df["y"].values)
    true_z_interp = np.interp(time_ms_vals, t_norm_ms, true_df["z"].values)

    err_mag = np.sqrt(
        (df["pos_x"].values - true_x_interp) ** 2 +
        (df["pos_y"].values - true_y_interp) ** 2 +
        (df["pos_z"].values - true_z_interp) ** 2
    )
    acc_err = np.cumsum(err_mag)

    fig_err, (ax_err_inst, ax_err_acc) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    fig_err.suptitle("Per-second Position Error Magnitude", fontsize=25, fontweight="bold")

    ax_err_inst.plot(time_ms_vals, err_mag, lw=1, color="steelblue")
    ax_err_inst.set_ylabel("Error magnitude (m)", fontsize=21)
    ax_err_inst.set_xlabel("Time (s)", fontsize=21)
    # ax_err_inst.set_title("Per-step position error magnitude", fontsize=25, fontweight="bold")
    ax_err_inst.grid(True, alpha=0.3)

    ax_err_acc.plot(time_ms_vals, acc_err, lw=1.2, color="firebrick")
    ax_err_acc.set_xlabel("Time (s)")
    ax_err_acc.set_ylabel("Accumulated error (m)")
    ax_err_acc.set_title("Accumulated position error magnitude")
    ax_err_acc.grid(True, alpha=0.3)

    # Mark GPS corrections every 400 s
    GPS_INTERVAL = 400
    t_max = time_ms_vals[-1]
    flag=False
    for ax in (ax_err_inst, ax_err_acc):
        t = GPS_INTERVAL
        while t <= t_max:
            ax.axvline(t, color="red", lw=1.2, ls="--", alpha=0.7)
            ax.text(t, 0.05, "GPS correction + Depth data",
                    transform=ax.get_xaxis_transform(),
                    rotation=90, va="bottom", ha="right",
                    fontsize=14, color="darkred", alpha=0.9)
            t += GPS_INTERVAL
            break


    fig_err.tight_layout()
    plt.savefig(os.path.join(SCRIPT_DIR, "kalman_error.png"), dpi=150, bbox_inches="tight")
    print(f"Saved kalman_error.png")
else:
    print("Skipping error plot — trueState.csv not available.")

plt.show()
