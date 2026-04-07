"""
Kalman Filter Test Results Visualiser
======================================
Reads generatedTest_results.csv produced by testFromFile() and plots:
  1. Estimated 3-D trajectory
  2. Position components over time  (x, y, z separate)
  3. Orientation quaternion over time
  4. Sensor-update breakdown (step counts by type)
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401

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

imu   = df[df["sensor"] == "IMU"]
depth = df[df["sensor"] == "DEPTH"]
gps   = df[df["sensor"] == "GPS"]

true_df = pd.read_csv(TRUE_FILE) if os.path.exists(TRUE_FILE) else None
if true_df is None:
    print("WARNING: trueState.csv not found — true trajectory will not be shown.")

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

# Mark GPS updates
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

# ── 3. Position components over step ─────────────────────────────────────────
def _mark_sensors(ax):
    """Add faint vertical lines where GPS measurements occur."""
    for s in gps["step"]:
        ax.axvline(s, color="orange", alpha=0.15, lw=0.6)

# True state uses time (t); estimated uses step index. Normalise true_df t to
# span [0, max_step] so both axes are comparable when overlaid.
true_step_scale = df["step"].max() / true_df["t"].max() if true_df is not None else 1.0

for ax, col, true_col, color, title in [
    (ax_x, "pos_x", "x", "royalblue",  "Position X over time"),
    (ax_y, "pos_y", "y", "seagreen",   "Position Y over time"),
    (ax_z, "pos_z", "z", "firebrick",  "Position Z (depth) over time"),
]:
    if true_df is not None:
        ax.plot(true_df["t"] * true_step_scale, true_df[true_col],
                lw=1, color="gray", ls="--", label="True")
    ax.plot(df["step"], df[col], lw=1, color=color, label="Estimated")
    if not gps.empty and col in ("pos_x", "pos_y"):
        ax.scatter(gps["step"], gps[col], color="orange", s=20, zorder=4, label="GPS update")
    _mark_sensors(ax)
    ax.set_xlabel("Step"); ax.set_ylabel("m")
    ax.set_title(title); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

# ── 4. Quaternion orientation ─────────────────────────────────────────────────
for comp, col in [("l (w)", "ori_l"), ("i", "ori_i"), ("j", "ori_j"), ("k", "ori_k")]:
    ax_q.plot(df["step"], df[col], lw=0.9, label=comp)
ax_q.axhline(0, color="black", lw=0.5, ls="--")
ax_q.set_xlabel("Step"); ax_q.set_ylabel("Component value")
ax_q.set_title("Orientation Quaternion over time")
ax_q.legend(fontsize=7); ax_q.grid(True, alpha=0.3)

# ── 5. Estimated velocity ─────────────────────────────────────────────────────
for comp, est_col, true_col, color in [("vx", "vel_a", "vx", "royalblue"),
                                        ("vy", "vel_b", "vy", "seagreen"),
                                        ("vz", "vel_c", "vz", "firebrick")]:
    if true_df is not None:
        ax_v.plot(true_df["t"] * true_step_scale, true_df[true_col],
                  lw=1, color=color, ls="--", alpha=0.5)
    ax_v.plot(df["step"], df[est_col], lw=1, label=comp, color=color)
_mark_sensors(ax_v)
ax_v.set_xlabel("Step"); ax_v.set_ylabel("m/s")
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
plt.show()
