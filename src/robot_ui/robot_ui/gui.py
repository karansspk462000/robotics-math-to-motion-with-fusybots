"""High-level Tkinter (ttkbootstrap) teleop/monitor UI for the robot stack.

Layout is entirely driven by robot_ui.config.RobotConfig (num_joints, joint
limits, topic names) - point system_config.yaml at a different robot (e.g.
an 18-DOF humanoid) and this UI's joint fields/readouts regenerate to match,
no code change needed.

Status display is pull-based, not a periodic timer: nothing here polls ROS
data on a schedule. "Sync to Current" pulls one snapshot and populates both
the target fields and the status panel from it; Send checks readiness from
a fresh snapshot only at the moment you click it. This is deliberate - nothing
in this UI's responsiveness depends on how fast hw_status/motion_status are
published.
"""
import math

import ttkbootstrap as ttk
from ttkbootstrap.constants import DANGER, INFO, PRIMARY, SECONDARY, SUCCESS, WARNING

from robot_ui.geometry import pose_to_flat_matrix

HW_STATUS_STALE_AFTER_SEC = 1.0


class RobotUiApp:
    def __init__(self, config, bridge):
        self.config = config
        self.bridge = bridge

        self.window = ttk.Window(themename="darkly")
        self.window.title(f"Robot UI - {config.robot_name} ({config.num_joints} DOF)")
        self.window.geometry("1180x720")

        self._build_header()
        body = ttk.Frame(self.window, padding=10)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        self._build_command_panel(body)
        self._build_status_panel(body)

        # One-shot, not a periodic timer: give ROS discovery a moment to
        # connect the subscriptions and receive a first hw_status, then
        # sync once automatically so the window doesn't open showing "press
        # Sync" when the backend is already running. If hw_status still
        # hasn't arrived (backend not up yet), retry once more after a
        # longer delay; after that, it's on the user to press Sync - this
        # never turns into a recurring poll.
        self.window.after(300, self._on_startup_sync)

    def _on_startup_sync(self):
        self._on_sync_to_current()
        if self.bridge.snapshot()["hw_status"] is None:
            self.window.after(1500, self._on_sync_to_current)

    # ------------------------------------------------------------------ header
    def _build_header(self):
        header = ttk.Frame(self.window, padding=(15, 10))
        header.pack(fill="x")

        ttk.Label(header, text=self.config.robot_name.upper(), font=("Segoe UI", 18, "bold")).pack(side="left")
        ttk.Label(header, text=f"  |  {self.config.num_joints} DOF  |  backend: {self.config.backend_mode}",
                  font=("Segoe UI", 11)).pack(side="left")

        self.connection_label = ttk.Label(header, text="UNKNOWN (press Sync)", font=("Segoe UI", 11, "bold"),
                                           bootstyle=WARNING)
        self.connection_label.pack(side="right")
        ttk.Label(header, text="Hardware link (as of last sync):  ", font=("Segoe UI", 11)).pack(side="right")

    # --------------------------------------------------------------- command
    def _build_command_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Send Command", padding=15)
        frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        # --- planning space ---
        space_row = ttk.Frame(frame)
        space_row.pack(fill="x", pady=(0, 10))
        ttk.Label(space_row, text="Planning space:", font=("Segoe UI", 10, "bold")).pack(side="left")
        self.space_var = ttk.StringVar(value="joint")
        ttk.Radiobutton(space_row, text="Joint", variable=self.space_var, value="joint",
                         command=self._on_space_changed, bootstyle="toolbutton").pack(side="left", padx=5)
        ttk.Radiobutton(space_row, text="Cartesian", variable=self.space_var, value="cartesian",
                         command=self._on_space_changed, bootstyle="toolbutton").pack(side="left", padx=5)

        # --- joint target entries ---
        self.joint_frame = ttk.Frame(frame)
        self.joint_vars = []
        for i, joint in enumerate(self.config.joints):
            row = ttk.Frame(self.joint_frame)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=f"Joint {i + 1}", width=10).pack(side="left")
            var = ttk.DoubleVar(value=0.0)
            ttk.Entry(row, textvariable=var, width=10).pack(side="left", padx=5)
            ttk.Label(row, text=f"[{joint.position_min:.2f}, {joint.position_max:.2f}] rad",
                      font=("Segoe UI", 8), bootstyle="secondary").pack(side="left")
            self.joint_vars.append(var)

        # --- cartesian target entries ---
        self.cartesian_frame = ttk.Frame(frame)
        self.cart_vars = {}
        for label, key, default in (("X (m)", "x", 0.0), ("Y (m)", "y", 0.0), ("Z (m)", "z", 0.5),
                                     ("Roll (rad)", "roll", 0.0), ("Pitch (rad)", "pitch", 0.0),
                                     ("Yaw (rad)", "yaw", 0.0)):
            row = ttk.Frame(self.cartesian_frame)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=label, width=10).pack(side="left")
            var = ttk.DoubleVar(value=default)
            ttk.Entry(row, textvariable=var, width=10).pack(side="left", padx=5)
            self.cart_vars[key] = var

        self.joint_frame.pack(fill="x")

        ttk.Separator(frame).pack(fill="x", pady=10)

        # --- planning method ---
        method_row = ttk.Frame(frame)
        method_row.pack(fill="x", pady=(0, 10))
        ttk.Label(method_row, text="Planning method:", font=("Segoe UI", 10, "bold")).pack(side="left")
        self.method_var = ttk.StringVar(value="velocity")
        for text, value in (("Velocity", "velocity"), ("Acceleration", "acceleration"), ("Time", "time")):
            ttk.Radiobutton(method_row, text=text, variable=self.method_var, value=value,
                             command=self._on_method_changed, bootstyle="toolbutton").pack(side="left", padx=5)

        constraints = ttk.Frame(frame)
        constraints.pack(fill="x")
        self.velocity_var = ttk.DoubleVar(value=self.config.default_velocity)
        self.acceleration_var = ttk.DoubleVar(value=self.config.default_acceleration)
        self.time_var = ttk.DoubleVar(value=2.0)

        self.velocity_entry = self._labeled_entry(constraints, "Max velocity (rad/s)", self.velocity_var)
        self.acceleration_entry = self._labeled_entry(constraints, "Max acceleration (rad/s^2)",
                                                        self.acceleration_var)
        self.time_entry = self._labeled_entry(constraints, "Desired time (s)", self.time_var)
        self._on_method_changed()

        ttk.Separator(frame).pack(fill="x", pady=10)

        buttons = ttk.Frame(frame)
        buttons.pack(fill="x")
        self.send_button = ttk.Button(buttons, text="Send Command", bootstyle=SUCCESS, command=self._on_send)
        self.send_button.pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Sync to Current", bootstyle=PRIMARY,
                   command=self._on_sync_to_current).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Send Home", bootstyle=INFO, command=self._on_send_home).pack(
            side="left", padx=(0, 8))
        ttk.Button(buttons, text="Send Zero Joints", bootstyle=SECONDARY, command=self._on_send_zero).pack(
            side="left")

        self.command_status_label = ttk.Label(
            frame, text="Press 'Sync to Current' to check the robot is reachable, or just Send - "
                        "it will refuse and tell you if hw_status has never arrived.",
            font=("Segoe UI", 9), bootstyle=WARNING, wraplength=380)
        self.command_status_label.pack(fill="x", pady=(10, 0))

    def _labeled_entry(self, parent, label, var):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label, width=22).pack(side="left")
        entry = ttk.Entry(row, textvariable=var, width=10)
        entry.pack(side="left", padx=5)
        return entry

    def _on_space_changed(self):
        if self.space_var.get() == "joint":
            self.cartesian_frame.pack_forget()
            self.joint_frame.pack(fill="x")
        else:
            self.joint_frame.pack_forget()
            self.cartesian_frame.pack(fill="x")

    def _on_method_changed(self):
        method = self.method_var.get()
        self.velocity_entry.configure(state="normal" if method == "velocity" else "disabled")
        self.acceleration_entry.configure(state="normal" if method in ("acceleration", "time") else "disabled")
        self.time_entry.configure(state="normal" if method == "time" else "disabled")

    # ---------------------------------------------------------------- status
    def _build_status_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Robot Status (updated by 'Sync to Current')", padding=15)
        frame.grid(row=0, column=1, sticky="nsew")

        ttk.Label(frame, text="Joint positions", font=("Segoe UI", 10, "bold")).pack(anchor="w")
        self.joint_bars = []
        self.joint_readouts = []
        for i, joint in enumerate(self.config.joints):
            row = ttk.Frame(frame)
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=f"J{i + 1}", width=4).pack(side="left")
            span = max(joint.position_max - joint.position_min, 1e-6)
            bar = ttk.Progressbar(row, maximum=span, length=180, bootstyle=PRIMARY)
            bar.pack(side="left", padx=5)
            readout = ttk.Label(row, text="- rad", width=14, font=("Consolas", 9))
            readout.pack(side="left")
            self.joint_bars.append((bar, joint.position_min, span))
            self.joint_readouts.append(readout)

        ttk.Separator(frame).pack(fill="x", pady=10)

        ttk.Label(frame, text="Cartesian pose (from robot_node FK)", font=("Segoe UI", 10, "bold")).pack(
            anchor="w")
        self.cartesian_readout = ttk.Label(frame, text="-", font=("Consolas", 10), justify="left")
        self.cartesian_readout.pack(anchor="w", pady=(4, 0))

        ttk.Separator(frame).pack(fill="x", pady=10)

        ttk.Label(frame, text="Digital twin (backend.mode: both only)", font=("Segoe UI", 10, "bold")).pack(
            anchor="w")
        self.digital_twin_readout = ttk.Label(frame, text="-", font=("Consolas", 9), bootstyle="secondary")
        self.digital_twin_readout.pack(anchor="w", pady=(4, 0))

    # --------------------------------------------------------------- actions
    def _on_send(self):
        try:
            snapshot = self.bridge.snapshot()
            if snapshot["hw_status"] is None:
                self.command_status_label.configure(
                    text="Refusing to send: hw_status has never arrived - is hw_interface_node running?",
                    bootstyle=DANGER)
                return
            age = snapshot["hw_status_age"]
            if age is None or age > HW_STATUS_STALE_AFTER_SEC:
                self.command_status_label.configure(
                    text=f"Refusing to send: hw_status is stale ({age:.1f}s old) - hardware link may be lost.",
                    bootstyle=DANGER)
                return

            planning_space = self.space_var.get()
            method = self.method_var.get()

            target_joint_position = []
            target_cartesian_pose = []
            if planning_space == "joint":
                target_joint_position = [v.get() for v in self.joint_vars]
                distance = self._joint_distance(target_joint_position, snapshot)
            else:
                pose = {k: v.get() for k, v in self.cart_vars.items()}
                target_cartesian_pose = pose_to_flat_matrix(pose["x"], pose["y"], pose["z"],
                                                             pose["roll"], pose["pitch"], pose["yaw"])
                distance = self._cartesian_distance(pose["x"], pose["y"], pose["z"], snapshot)

            acceleration = self.acceleration_var.get()
            if method == "time":
                velocity = self._velocity_for_time(distance, self.time_var.get(), acceleration)
                self.velocity_var.set(round(velocity, 4))
            else:
                velocity = self.velocity_var.get()

            self.bridge.send_user_command(planning_space, target_joint_position, target_cartesian_pose,
                                           velocity, acceleration)
            self.command_status_label.configure(
                text=f"Sent {planning_space} command (v={velocity:.3f}, a={acceleration:.3f})",
                bootstyle=SUCCESS)
        except Exception as exc:  # noqa: BLE001 - surface any bad input to the operator, don't crash the UI
            self.command_status_label.configure(text=f"Failed to send command: {exc}", bootstyle=DANGER)

    def _on_send_home(self):
        for var, position in zip(self.joint_vars, self.config.home_joint_position):
            var.set(position)
        self.space_var.set("joint")
        self._on_space_changed()
        self._on_send()

    def _on_send_zero(self):
        for var in self.joint_vars:
            var.set(0.0)
        self.space_var.set("joint")
        self._on_space_changed()
        self._on_send()

    def _on_sync_to_current(self):
        """Pulls one snapshot and updates everything from it: the target
        fields (so an edit starts from where the robot actually is), the
        status panel, and the connection indicator. This is the only place
        any of those get refreshed - there is no periodic timer."""
        snapshot = self.bridge.snapshot()
        hw_status = snapshot["hw_status"]
        motion_status = snapshot["motion_status"]

        if hw_status is not None and len(hw_status.joint_positions) == len(self.joint_vars):
            for var, position in zip(self.joint_vars, hw_status.joint_positions):
                var.set(round(position, 4))

        if motion_status is not None and len(motion_status.cartesian_pose) == 6:
            x, y, z, roll, pitch, yaw = motion_status.cartesian_pose
            self.cart_vars["x"].set(round(x, 4))
            self.cart_vars["y"].set(round(y, 4))
            self.cart_vars["z"].set(round(z, 4))
            self.cart_vars["roll"].set(round(roll, 4))
            self.cart_vars["pitch"].set(round(pitch, 4))
            self.cart_vars["yaw"].set(round(yaw, 4))

        self._update_connection_indicator(snapshot)
        self._update_joint_status(snapshot)
        self._update_cartesian_status(snapshot)
        self._update_digital_twin_status(snapshot)

        if hw_status is None and motion_status is None:
            self.command_status_label.configure(text="No status received yet - nothing to sync from.",
                                                  bootstyle=WARNING)
        else:
            self.command_status_label.configure(text="Target fields and status synced to current robot state.",
                                                  bootstyle=INFO)

    def _joint_distance(self, target, snapshot):
        hw_status = snapshot["hw_status"]
        if hw_status is None or len(hw_status.joint_positions) != len(target):
            return max(abs(v) for v in target) if target else 0.0
        return max(abs(t - c) for t, c in zip(target, hw_status.joint_positions))

    def _cartesian_distance(self, x, y, z, snapshot):
        motion_status = snapshot["motion_status"]
        if motion_status is None or len(motion_status.cartesian_pose) != 6:
            return math.sqrt(x * x + y * y + z * z)
        cx, cy, cz, *_ = motion_status.cartesian_pose
        return math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2)

    @staticmethod
    def _velocity_for_time(distance, desired_time, acceleration):
        """Inverts the trapezoidal-profile time equation T = v/a + D/v for v,
        given a fixed acceleration - see trajectory_planner's
        TrapezoidalTrajectoryGenerator for the profile this approximates.
        """
        if desired_time <= 0 or acceleration <= 0:
            return 0.0
        discriminant = (acceleration * desired_time) ** 2 - 4 * acceleration * distance
        if discriminant < 0:
            # desired_time too short to honor the profile at this
            # acceleration - fall back to a straight ramp estimate.
            return 2.0 * distance / desired_time if desired_time > 0 else 0.0
        return (acceleration * desired_time - math.sqrt(discriminant)) / 2.0

    # ------------------------------------------------------ status (pull-only)
    def _update_connection_indicator(self, snapshot):
        age = snapshot["hw_status_age"]
        is_live = age is not None and age < HW_STATUS_STALE_AFTER_SEC
        self.connection_label.configure(text="LIVE" if is_live else "NO STATUS",
                                         bootstyle=SUCCESS if is_live else DANGER)

    def _update_joint_status(self, snapshot):
        hw_status = snapshot["hw_status"]
        if hw_status is None:
            return
        for i, position in enumerate(hw_status.joint_positions):
            if i >= len(self.joint_bars):
                break
            bar, position_min, span = self.joint_bars[i]
            bar["value"] = max(0.0, min(span, position - position_min))
            self.joint_readouts[i].configure(text=f"{position:+.3f} rad")

    def _update_cartesian_status(self, snapshot):
        motion_status = snapshot["motion_status"]
        if motion_status is None or len(motion_status.cartesian_pose) != 6:
            return
        x, y, z, roll, pitch, yaw = motion_status.cartesian_pose
        self.cartesian_readout.configure(
            text=f"X: {x:+.4f}  Y: {y:+.4f}  Z: {z:+.4f}\nRoll: {roll:+.4f}  Pitch: {pitch:+.4f}  Yaw: {yaw:+.4f}")

    def _update_digital_twin_status(self, snapshot):
        twin_status = snapshot["digital_twin_status"]
        age = snapshot["digital_twin_status_age"]
        if twin_status is None or age is None or age > HW_STATUS_STALE_AFTER_SEC:
            self.digital_twin_readout.configure(text="(no digital_twin_status - not in 'both' mode)")
            return
        positions = ", ".join(f"{p:+.3f}" for p in twin_status.joint_positions)
        self.digital_twin_readout.configure(text=f"[{positions}]")

    def run(self):
        self.window.mainloop()
