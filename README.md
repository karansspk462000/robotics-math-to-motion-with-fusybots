# robotics-math-to-motion-with-fusybots

This Repository contains a generalized Robotics Codebase which you can scale for any Robot. If you are a beginner in Robotics and want to learn how to write robotics code you can explore this and also explore my youtube channel for detailed explanation.

## Workspace: `robot_fundamentals_ws`

A ROS2 (Jazzy) C++ workspace for a 6-axis robot arm pipeline: motion planning,
kinematics, trajectory generation, hardware/digital-twin backends, and
visualization, built package-by-package around the Strategy pattern so any
stage (kinematics method, path planner, interpolator, trajectory generator,
hardware backend) can be swapped via config, not code changes.

See [`docs/ARCHITECTURE_LECTURE_NOTES.md`](docs/ARCHITECTURE_LECTURE_NOTES.md)
for a full walkthrough of the design patterns, SOLID principles, C++
idioms, concurrency model, and the underlying robotics (DH parameters,
closed-form IK, Jacobians, singularities, trajectory profiles) used
throughout, with concrete references into this codebase.

## Pipeline

```
robot_ui / any client
      | /user_command
      v
robot_node (state_orchestrator) -- owns MotionPlanner, RobotStateMachine
      | /hw_command                              ^ /hw_status
      v                                           |
hw_interface_node  <-- or -->  mujoco_digital_twin_node
      |
      v
joint_state_bridge_node -> /joint_states -> robot_state_publisher -> RViz
```

`MotionPlanner` (pure C++, no ROS) composes, per motion request:
`kinematics` (FK/IK) -> `path_planner` -> `path_interpolator` ->
`trajectory_planner`, each stage injected as an interface so any one of them
can be swapped independently.

## Packages

| Package | Role |
|---|---|
| `kinematics` | FK/IK strategies (analytical, KDL) + Context wrappers (`ForwardKinematics`/`InverseKinematics`) that allow runtime strategy swap |
| `path_planner` | Joint-space sparse waypoint planning - `rrt_star` (real RRT* algorithm). Takes an injectable `IsStateValidFn` (collision checking - see `collision_checking`) and `EdgeCostFn` (optimization, e.g. `jointLimitMarginCostFn`), both optional and decoupled - this package depends on neither `collision_checking` nor `kinematics`. Cartesian-space planning doesn't use this package at all - see `motion_planner` |
| `collision_checking` | `ICollisionChecker` + `PlanningScene` (Context): the live, queryable "what's in the world" model. Phase 1 only - static box/sphere obstacles, checked against the end-effector position via `kinematics::ForwardKinematics`. No self-collision, no real-time vision updates yet (see Future scope) |
| `path_interpolator` | Densifies a sparse path (joint-space lerp, Cartesian lerp+slerp) |
| `trajectory_planner` | Time-parameterizes a dense path into a trapezoidal-velocity trajectory |
| `motion_planner` | Composition root wiring the four stages above into one `plan()` call - pure C++, ROS-free, unit-testable |
| `robot_hw_interface` | `IRobotInterface` + `DualRobotInterface` (composite: mirrors commands to hardware + digital twin) |
| `hw_interface` | ROS boundary node publishing `/hw_status`, consuming `/hw_command`, backed by a hardware/dummy strategy |
| `digital_twin` | Kinematic-only C++ digital twin backend (see `mujoco_digital_twin` for the physics-accurate Python alternative) |
| `mujoco_digital_twin` | Real MuJoCo-physics digital twin, drop-in alternative to `hw_interface_node` - no MJCF model ships with this repo yet for any robot, see that package's `main.py` for the expected format and path |
| `state_orchestrator` | `robot_node` (ROS orchestrator: subscriptions/publishers/timers, drives `MotionPlanner`, owns `RobotStateMachine`), `state_manager/` (readiness/fault tracking), `visualization/` (`/hw_status` -> `/joint_states` bridge for RViz) |
| `robot_config` | Loads `system_config.yaml` (which strategy per stage, topic names, timing, robot description file) into a typed `SystemConfig` |
| `robot_description`, `dynamics` | URDF/meshes, dynamics utilities |
| `math_utils` | Shared Eigen/KDL/pose/quaternion conversion helpers - purely generic math, no robot-domain knowledge (renamed from `robot_utils`, since that's all it ever contained) |
| `robot_ui` | Tkinter teleop UI |
| `integration_tests` | gtest suites, one file per architectural area (see below) |

## Configuration

Everything pipeline-stage-specific and every periodic rate lives in one file:
`robot_config/config/system_config.yaml`. Key sections:

- Top-level: which strategy each stage uses (`kinematics_method`,
  `path_planner_method`, `trajectory_planner_method`, ...).
- `backend:` which hardware/digital-twin backend(s) to run and their params.
- `timing:` every periodic rate in one place - `hw_status_publish_rate_hz`,
  `trajectory_playback_rate_hz` (also drives the trajectory generator's own
  sample spacing, so the two can never drift apart), `state_machine_poll_rate_hz`,
  `mujoco_physics_timestep_sec`.
- `state_machine:` hardware-link timeout and hw/twin divergence threshold.

## Building and testing

```bash
colcon build
colcon test --packages-select integration_tests
```

`integration_tests` is split by architectural area rather than one giant
file:

- `test_kinematics` - FK/IK round-tripping, Jacobian cross-validation, home-pose singularity checks. Links only `kinematics`.
- `test_trajectory_planner` - pure trajectory-generation logic (e.g. per-joint velocity bottleneck). Links only `trajectory_planner`.
- `test_path_planner` - `RrtStarPathPlanner` in isolation: reaches the goal, respects bounds, throws on invalid start/goal, is deterministic given a fixed seed, avoids an injected obstacle a straight line would have crossed, and (the `jointLimitMarginCostFn` optimization hook) penalizes near-limit states correctly while a zero weight is a true no-op. Links only `path_planner`.
- `test_collision_checking` - `PlanningScene` in isolation with real KDL kinematics: empty scene is always valid, box/sphere obstacles are detected and correctly cleared by `removeObject`, a far-away obstacle doesn't affect validity.
- `test_motion_planner` - full-pipeline regressions (straight-line Cartesian motion, velocity-profile consistency) through the real composition root, plus confirming the Cartesian branch's collision pre-check hook actually gates planning.
- `test_robot_state_machine` - unit tests for the safety-critical readiness/fault state machine, in isolation.

### Prototyping notebook

[`integration_tests/notebooks/fk_ik_dh_sandbox.ipynb`](src/integration_tests/notebooks/fk_ik_dh_sandbox.ipynb)
is a standalone Python (numpy + sympy + matplotlib) sandbox for the `analytical`
kinematics strategy - symbolic and numeric DH-chain FK, the full closed-form IK
derivation, all 8 IK solutions for a given pose (with a 3D plot comparing two of
them), the FK→IK→FK round-trip check, a singularity-clamping demo, and a
trapezoidal-trajectory bonus section - each formula a verified 1:1 match to the
real C++ in `kinematics`/`trajectory_planner`. Not part of the `colcon build`
(it's a notebook, not a package target); run it with `pip install numpy sympy
matplotlib jupyter` then `jupyter notebook`. Intended workflow: work out any
kinematics/trajectory math change here first, confirm it against the
assertions in the notebook, then port it to the C++ strategy and its
regression test in `integration_tests/test/test_kinematics.cpp`.
