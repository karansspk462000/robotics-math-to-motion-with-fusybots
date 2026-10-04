"""Regenerates docs/images/dh_frame_overview.png and dh_wrist_zoom.png.

Run from the workspace root: python3 docs/images/generate_dh_diagrams.py
Re-run this after changing link lengths (keep in sync with
fusybots_six_dof_spherical_wrist.yaml / kinematics/common/six_axis_link_lengths.hpp) or
the demo pose - the same numeric DH logic used here is verified against the
real C++ in integration_tests/notebooks/fk_ik_dh_sandbox.ipynb.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Exactly matching fusybots_six_dof_spherical_wrist.yaml / kinematics/common/six_axis_link_lengths.hpp
l1, d2, l3, l5, l6, l7, le = 0.2, 0.01, 0.3, 0.32, 0.25, 0.0, 0.10


def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1],
    ])


def compute_all_frames(theta):
    """All 7 frames F0..F7 including the fixed tool row - mirrors
    AnalyticalSixAxisForwardKinematics::computeFk(7, theta) but returns every
    intermediate frame, not just the final one."""
    alpha = [0, np.pi / 2, 0, 0, -np.pi / 2, np.pi / 2, 0]
    a = [0, 0, -l3, -l5, 0, 0, 0]
    d = [l1, 0, d2, l6, l7, 0, le]

    adjusted = list(theta)
    adjusted[0] += np.pi / 2
    adjusted[1] -= np.pi / 2
    adjusted[3] += np.pi / 2
    adjusted[5] += np.pi / 2
    adjusted = adjusted + [-np.pi]  # fixed tool row

    frames = [np.eye(4)]
    for i in range(7):
        frames.append(frames[-1] @ dh_transform(adjusted[i], d[i], a[i], alpha[i]))
    return frames


def draw_triad(ax, T, length=0.05, label=None, label_offset=(0.01, 0.01, 0.01), label_color='black'):
    origin = T[0:3, 3]
    R = T[0:3, 0:3]
    colors = ['r', 'g', 'b']  # x, y, z  (z = blue = the joint's own rotation axis)
    for i, c in enumerate(colors):
        axis_dir = R[:, i] * length
        ax.quiver(*origin, *axis_dir, color=c, linewidth=1.6, arrow_length_ratio=0.3)
    if label:
        ax.text(origin[0] + label_offset[0], origin[1] + label_offset[1],
                 origin[2] + label_offset[2], label, fontsize=9, weight='bold', color=label_color)


def draw_skeleton(ax, frames, joint_color='k'):
    pts = np.array([f[0:3, 3] for f in frames])
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-', color=joint_color, linewidth=2.5, zorder=1)
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color='dimgray', s=25, zorder=2)


def set_equal_aspect_3d(ax, frames, pad=0.05):
    pts = np.array([f[0:3, 3] for f in frames])
    mins, maxs = pts.min(axis=0) - pad, pts.max(axis=0) + pad
    center = (mins + maxs) / 2
    radius = max(maxs - mins) / 2
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


# Representative non-singular pose (degrees -> radians), chosen so no two
# consecutive frames coincide or overlap in the rendering.
theta_deg = [20, 35, -45, 15, 40, 25]
theta = [np.deg2rad(v) for v in theta_deg]
frames = compute_all_frames(theta)
labels = ["F0 (base)", "F1", "F2", "F3", "F4 (wrist ctr)", "F5", "F6", "F7 (EE/tool)"]

# Label offsets: hand-picked so labels never overlap even where frames
# coincide (F1/F2 at one point; F4/F5/F6 at another) - the OFFSETS are
# purely cosmetic (for label legibility), the arrow lengths are NOT: every
# triad below is drawn at the same physical length, since the coincidence
# itself (not a length difference) is the thing being illustrated.
label_offsets = [
    (0.015, 0.015, -0.02),   # F0
    (-0.05, -0.02, 0.03),    # F1  (coincides with F2 - pushed down/left)
    (0.02, 0.03, 0.04),      # F2  (coincides with F1 - pushed up/right)
    (0.02, 0.02, -0.03),     # F3
    (-0.07, 0.0, 0.05),      # F4  (coincides with F5, F6 - pushed left)
    (0.0, 0.05, -0.02),      # F5  (coincides with F4, F6 - pushed up)
    (0.0, -0.06, 0.02),      # F6  (coincides with F4, F5 - pushed down)
    (0.02, 0.02, 0.02),      # F7
]
label_colors = ['black', 'purple', 'purple', 'black', 'darkred', 'darkred', 'darkred', 'black']

# ---------------- Figure 1: full-arm overview ----------------
fig = plt.figure(figsize=(10, 8.5))
ax = fig.add_subplot(111, projection='3d')
draw_skeleton(ax, frames)
for i, f in enumerate(frames):
    draw_triad(ax, f, length=0.06, label=labels[i], label_offset=label_offsets[i], label_color=label_colors[i])
set_equal_aspect_3d(ax, frames)
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
ax.set_title(f"DH frame chain, theta = {theta_deg} deg\n"
             "red=x  green=y  blue=z (= that joint's own rotation axis)\n"
             "purple labels (F1,F2) share one origin; dark-red labels (F4,F5,F6) share another (spherical wrist)")
plt.tight_layout()
fig.savefig("docs/images/dh_frame_overview.png", dpi=150)
print("wrote dh_frame_overview.png")

# ---------------- Figure 2: wrist zoom (F4, F5, F6, F7) ----------------
fig2 = plt.figure(figsize=(8.5, 7.5))
ax2 = fig2.add_subplot(111, projection='3d')
wrist_frames = frames[4:8]
wrist_labels = labels[4:8]
wrist_offsets = label_offsets[4:8]
wrist_colors = label_colors[4:8]
draw_skeleton(ax2, wrist_frames, joint_color='darkorange')
for i, f in enumerate(wrist_frames):
    draw_triad(ax2, f, length=0.04, label=wrist_labels[i], label_offset=wrist_offsets[i], label_color=wrist_colors[i])
set_equal_aspect_3d(ax2, wrist_frames, pad=0.05)
ax2.set_xlabel('x (m)'); ax2.set_ylabel('y (m)'); ax2.set_zlabel('z (m)')
ax2.set_title("Spherical wrist zoom: F4, F5, F6 share one origin\n"
              "(only F7's tool offset 'le' extends beyond that point)")
plt.tight_layout()
fig2.savefig("docs/images/dh_wrist_zoom.png", dpi=150)
print("wrote dh_wrist_zoom.png")

# sanity: confirm F4, F5, F6 origins really do coincide (spherical wrist proof, visually)
o4, o5, o6 = frames[4][0:3, 3], frames[5][0:3, 3], frames[6][0:3, 3]
print("max origin spread among F4/F5/F6:", max(np.linalg.norm(o4 - o5), np.linalg.norm(o5 - o6), np.linalg.norm(o4 - o6)))
