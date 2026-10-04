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
| `mujoco_digital_twin` | Real MuJoCo-physics digital twin, drop-in alternative to `hw_interface_node` |
| `state_orchestrator` | `robot_node` (ROS orchestrator: subscriptions/publishers/timers, drives `MotionPlanner`, owns `RobotStateMachine`), `state_manager/` (readiness/fault tracking), `visualization/` (`/hw_status` -> `/joint_states` bridge for RViz) |
| `robot_config` | Loads `system_config.yaml` (which strategy per stage, topic names, timing, robot description file) into a typed `SystemConfig` |
| `robot_description`, `dynamics`, `mesh_align_tool` | URDF/meshes, dynamics utilities, mesh alignment tooling |
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

## Recent architectural work (this session)

1. **FK/IK Context symmetry** - `ForwardKinematics` now goes through the same
   Context wrapper pattern `InverseKinematics` already used, instead of
   `MotionPlanner`/`RobotNode` holding a raw `IFKStrategy` for FK only.
2. **Layer-based folder structure** - `kinematics`, `path_planner`,
   `path_interpolator`, `trajectory_planner`, `robot_hw_interface`,
   `hw_interface`, `digital_twin` now organize `include/<pkg>/` and `src/`
   into `interface/`, `strategy/`, `factory/` (plus `context/`/`common/`
   where applicable) subfolders.
3. **Centralized timing config** - every periodic rate moved into
   `system_config.yaml`'s `timing:` block; the trajectory generator's sample
   spacing is now *derived from* the same rate that drives the playback
   timer, closing off a whole class of "reported velocity doesn't match
   real motion" bugs.
4. **`state_orchestrator` reorg + rename** - `RobotPlannerNode`/
   `robot_planner_node` renamed to `RobotNode`/`robot_node` (planning
   happens in `motion_planner`, not here); `robot_state_machine` moved under
   `state_manager/`; `joint_state_bridge_node` moved under `visualization/`.
5. **Naming cleanup** - the `fk_strategy_` member (already holding the
   `ForwardKinematics` Context, not a raw strategy) renamed to
   `forward_kinematics_` for consistency with `inverse_kinematics_`.
6. **`RobotStateMachine` hardening** - deleted copy/move constructors and
   assignment operators; it's safety-critical latched state meant to have
   exactly one live instance, so an accidental copy is now a compile error
   instead of a silent duplicate-state hazard.
7. **Test suite split + new coverage** - `integration_tests`' single
   578-line file replaced with 4 focused binaries (see above);
   `RobotStateMachine`, which previously had zero test coverage, now has 8
   unit tests covering transitions, timeout/fault/recovery, and divergence
   warnings.
8. **`RobotStateMachine` migrated to the GoF State pattern** -
   `state_manager/interface/robot_state_interface.hpp` (`IRobotState`) plus
   one class per state under `state_manager/states/` (`StartingRobotState`,
   `SoftwareUpState`, `ReadyStateBase` + its three thin leaves,
   `HardwareLinkLostState`), replacing the previous enum+switch transition
   logic. Done in anticipation of humanoid/fleet support needing many more
   states with more divergent behavior - adding a new state is now "add one
   class," not "edit every existing transition method." All 8 existing
   `RobotStateMachine` unit tests pass unchanged (public API untouched), and
   a live fault/recovery cycle (killing and restarting `hw_interface_node`)
   was re-verified end-to-end.
9. **State classes consolidated** - the 7 `IRobotState` implementations
   (14 files: 7 headers + 7 sources) merged into one
   `state_manager/states/robot_states.{hpp,cpp}` pair - most were one-method
   overrides, so a class-per-file split was pure navigation overhead.
   Existing state classes still never need editing to add a new one (OCP is
   preserved); a new state now means appending to this file rather than
   adding a new file.
10. **`robot_utils` renamed to `math_utils`** - every function in the
    package was pure math/conversion (Eigen/std::vector, Eigen/KDL, pose
    packing, quaternion normalization) with zero robot-domain knowledge, so
    the name was misleading. Also reorganized from a flat header list into
    `eigen/`, `kdl/`, `pose/` subfolders matching what each header bridges.
11. **Concurrency audit** - full review of every mutex/lock/thread in the
    workspace (see `docs/ARCHITECTURE_LECTURE_NOTES.md` §5 for the full
    writeup). Headline finding: `RobotNode`/`HwInterfaceNode` both run
    under `rclcpp::spin`'s default single-threaded executor, so their
    mutexes are currently insurance, not load-bearing - real, necessary
    locking exists on the Python side (`robot_ui`, `mujoco_digital_twin`,
    both genuinely multi-threaded). Fixed two latent issues in
    `robot_node.cpp` found during the audit: a TOCTOU gap in
    `userCommandCallback` (now re-checks `isReady()` in the same locked
    scope that commits `active_trajectory_`), and inconsistent lock scope
    around `publish()` in `trajectoryPlaybackTimerCallback` (now always
    publishes after the lock releases, in all three branches).
12. **`LinearPathPlanner` removed from `path_planner`; replaced with a real
    `RrtStarPathPlanner`.** `linear`/cubic/curve are interpolation concepts
    (`path_interpolator` already owns those) - a "path planner" whose whole
    job was `return {start, goal}` was never really planning anything, and
    having it in `path_planner` at all was a category-confusion worth
    removing outright, not just renaming. `RrtStarPathPlanner` is a real
    algorithm (nearest-neighbor steering, cost-aware parent selection,
    rewiring with cost propagation to descendants), with collision-checking
    injected as a `std::function` defaulting to "always valid" (this
    workspace has no obstacle model yet - that's the deliberately dummy
    half; the algorithm itself is real). `path_planner_method: "rrt_star"`
    is now the only option and the real default. `MotionPlanner`'s
    Cartesian-space branch never actually needed a "planner" (there's no
    Cartesian obstacle model), so it now builds its sparse `{current,
    target}` path directly instead of calling `path_planner_` - which
    means `path_planner_` is joint-space-only now, cleanly, with no
    dimension-mismatch risk. 6 new unit tests in `test_path_planner`,
    including one that actually proves the algorithm avoids an injected
    obstacle a straight line would have crossed.
13. **Found and fixed a real latency bug this surfaced**: this workspace
    has always built without optimization (`CMAKE_BUILD_TYPE` empty). That
    was invisible before because every existing algorithm here is cheap
    regardless of optimization level - RRT* is the first real computation,
    and unoptimized it took ~1.1s per plan (confirmed by direct
    instrumentation of `MotionPlanner::planJointSpace`), long enough to
    blow through `hw_status_timeout_sec` (0.5s) and trigger a false
    hardware-fault live. Added `colcon_defaults.yaml`
    (`build: {cmake-args: [-DCMAKE_BUILD_TYPE=RelWithDebInfo]}`) so a plain
    `colcon build` now optimizes by default (confirmed: `~/.colcon`'s
    `colcon-defaults` extension picks this up automatically) - brought the
    same plan down to ~11ms, a 100x difference, verified via direct
    benchmarking, the full test suite (`test_path_planner`: 7.6s → 109ms),
    and a clean live run with no fault. `RelWithDebInfo` (not `Release`)
    was chosen specifically to keep full debug symbols for `gdb`/`lldb`
    while still optimizing.
14. **Fixed a duplicate-final-waypoint edge case** in `RrtStarPathPlanner`
    found while benchmarking: `planPath` used to unconditionally append
    `goal` even when the tree's own last node already landed exactly on it
    (common, since `goal_bias` makes sampling `goal` itself frequent) -
    producing a zero-length final segment that `path_interpolator` then
    wastefully densified into `points_per_segment` copies of one point.
    Now only appends `goal` if the path doesn't already end there.
15. **Added `collision_checking` package** (`ICollisionChecker` +
    `PlanningScene`, Phase 1: static box/sphere obstacles checked against
    the end-effector position via `ForwardKinematics`) - the seam for real
    collision avoidance without `path_planner`/`motion_planner` depending
    on it directly (both take a generic `std::function` instead; the
    composition root, `robot_node_main.cpp`, is the only place that knows
    about both). `PlanningScene` is a Context (shared_ptr, mutex-guarded)
    for the same reason `ForwardKinematics` is: multiple independent
    owners (`path_planner`'s RRT*, `motion_planner`'s Cartesian pre-check,
    a future vision-fed updater) observe/will-mutate the same live scene.
    Wired into the real pipeline (empty scene today - no obstacle-authoring
    format exists yet, so this is honestly still "dummy" data through real
    infrastructure, not a stub function). 6 new unit tests in
    `test_collision_checking`.
16. **Added an optimization hook to `RrtStarPathPlanner`**: a pluggable
    `EdgeCostFn` (defaults to plain Euclidean distance, preserving prior
    behavior) that lets the search prefer paths that stay away from joint
    limits (`jointLimitMarginCostFn`) without rejecting near-limit states
    outright - validity and cost answer different questions (allowed vs.
    how much to avoid it). Wired into the real pipeline with a modest
    default weight. A similar singularity-avoidance cost term (via a
    manipulability measure from `kinematics::InverseKinematics::
    computeJacobian`) would plug in the same way, built as a closure at the
    composition root - deliberately not built this round, since it needs
    real calibration work of its own. `motion_planner::MotionPlanner` also
    gained an optional `IsJointStateValidFn` pre-check hook for its
    Cartesian branch (which never calls `path_planner_` at all - see item
    12 above), so a Cartesian move's start/resolved-IK-target are checked
    against the same `PlanningScene` even though the dense interpolated
    path between them isn't. 3 new unit tests (2 for the cost function
    directly, 1 for the pre-check hook).

All changes verified with a full `colcon build` (19/19 packages), full
`colcon test` (6/6 binaries, 36/36 tests), and live pipeline runs
(`hw_interface_node` + `robot_node`) confirming node names, `/hw_command`
publish rate, end-to-end joint- and Cartesian-space motion, and
hardware-link-loss/recovery behavior.

## Future scope (discussed, deferred - not yet implemented)

Real architectural directions raised and discussed, deliberately not acted
on yet because none has a concrete, current trigger - revisit each when its
trigger actually arrives rather than building against a guess:

1. **`/user_command` as a ROS2 Action instead of a plain topic.** Today a
   client has no way to know if a move was accepted, is progressing,
   succeeded, failed, or how to cancel it mid-flight - it can only watch
   `/motion_status` separately and guess. This is exactly what ROS2 Actions
   (goal/feedback/result/cancel) exist for, and Services are a poor fit
   for anything long-running. **Highest-value, most overdue item on this
   list** - it fixes a real, current gap rather than anticipating a
   hypothetical one, so it's the natural first pick if/when this list gets
   picked up.
2. **Logging & telemetry.** Currently ad hoc `RCLCPP_INFO/WARN/ERROR` calls
   with no persistence and no consistent severity conventions. Don't build
   a custom telemetry pipeline - `ros2 bag record` on `/hw_status`,
   `/hw_command`, `/motion_status`, `/user_command` gives replayable
   telemetry for free, and `rclcpp`'s existing per-node log-severity
   config covers debug-level logging. Low-risk, cheap to start whenever.
3. **Profiling.** No timing instrumentation exists around
   `MotionPlanner::plan()` or callback execution vs. the configured
   `timing:` rates. Prefer `ros2_tracing`/`tracetools` (the purpose-built
   LTTng-based tool for an `rclcpp` stack) over a custom profiler; a
   lightweight RAII `ScopedTimer` in `math_utils`, logged at DEBUG level,
   is a cheap in-house alternative if a full tracing setup is overkill.
4. **Task manager / Behavior Tree layer.** Today one `/user_command` is
   exactly one trajectory - no multi-step task composition, retries, or
   reactive branching. If/when a real multi-step task exists (pick → move
   → place, etc.), a BT (`BehaviorTree.CPP`) should sit **above**
   `robot_node` as its own node, driving it through whatever interface it
   exposes (see item 1) - not be baked into `RobotNode` itself. Do not
   build this speculatively; it's exactly the premature-abstraction trap
   this workspace has otherwise avoided.
5. **Node lifecycle management (`rclcpp_lifecycle`).** Distinct from
   `RobotStateMachine`: lifecycle governs whether *the node's own software*
   is initialized (configure/activate/deactivate/shutdown), a supervisor's
   concern; `RobotStateMachine` governs whether *the physical robot* is
   ready, assuming the node is already up. Worth adopting once something
   *external* (a fleet supervisor, a launch orchestrator) needs
   deterministic, remotely-triggered bring-up/teardown of nodes - with a
   single node today, there's no supervisor to drive it yet.
6. **`collision_checking` Phase 2**: real self-collision and real
   environment-collision, both properly geometry-based instead of Phase
   1's single-point (end-effector-only) approximation. Two things this
   needs, and they're separate design questions:
   - **The robot's own footprint (for self-collision)**: either (a) static
     per-link collision geometry - boxes/capsules/convex hulls derived from
     each link's URDF `<collision>` tag, which doesn't exist for any robot
     in this workspace yet, or (b) a live, vision-derived shape (e.g. a
     point cloud/voxel model of the arm itself, useful as a check against
     calibration drift or unmodeled attachments a static URDF model
     wouldn't know about). These aren't mutually exclusive - a real system
     often wants (a) as the default, fast path with (b) as a periodic
     cross-check.
   - **The collision-checking engine itself**: Phase 1's `isPointInsideObject`
     is a hand-rolled point-vs-primitive test, deliberately minimal. Real
     self-collision (link-vs-link, both moving) and real mesh/primitive
     environment collision need a proper narrow-phase collision library -
     **FCL (Flexible Collision Library)** is the standard choice here (it's
     what MoveIt itself uses under the hood), handling both shape-vs-shape
     and full self-collision-pair matrices properly, including broad-phase
     pruning so checking every link against every obstacle every query
     doesn't become the bottleneck. `PlanningScene` would wrap FCL rather
     than reimplementing narrow-phase geometry - the `ICollisionChecker`
     interface shouldn't need to change to accommodate this.

   Also needed regardless of the above: a vision package updating
   `PlanningScene` in real time via `addObject`/`removeObject`, and a
   runtime collision monitor (checking the robot isn't about to hit
   something *while it's moving*, fed by `/hw_status`). The runtime
   monitor is conceptually close to `RobotStateMachine`'s fault-handling
   idea, but should likely be a *separate* concern coordinated by
   `RobotNode` rather than folded into the same state machine - a decision
   to make when this is actually built, not now. An obstacle-authoring
   format (a new `obstacles:` block in `system_config.yaml`, or a
   dedicated scene file) is also needed before `PlanningScene` can hold
   anything in the real running system - today it's always constructed
   empty.
7. **Singularity-avoidance cost for `RrtStarPathPlanner`.** The
   `EdgeCostFn` extensibility point exists (see `jointLimitMarginCostFn`);
   a manipulability-based term (via
   `kinematics::InverseKinematics::computeJacobian` - e.g. penalizing low
   Jacobian condition number / determinant) would plug in the exact same
   way, built as a closure at the composition root capturing the real
   `InverseKinematics` Context. Not built yet because it needs real
   calibration (how much penalty, over what manipulability range) that's
   better done against an actual observed problem than guessed at.
8. **`MotionPlanner`'s Cartesian branch only checks joint-space validity at
   the two endpoints (current + resolved-IK-target), not every dense
   interpolated waypoint in between** - a real gap, flagged directly: today
   a Cartesian move could pass through a joint-limit violation or collision
   *mid-path* undetected. The fix is **not** "build a second `RrtStarPathPlanner`
   configured for Cartesian-space bounds" - that would either (a) throw away
   the Cartesian straight-line guarantee this project already fixed once as
   a historical bug (`EndEffectorFollowsStraightLine`; RRT* in joint space
   does not produce a straight Cartesian line), or (b) need a second,
   parallel notion of "bounds" (workspace X/Y/Z reach + quaternion range)
   that means something different from joint limits, which is exactly the
   "two things for one algorithm, just because the input changed" smell to
   avoid. The right fix keeps the Cartesian-space interpolation geometry
   exactly as-is (that part is correct and shouldn't change) and instead
   extends the *existing* joint-space `IsJointStateValidFn` check - already
   used for both the joint-space planner and the two Cartesian endpoints -
   to run against **every** waypoint in `planCartesianSpace`'s
   IK-resolved `joint_path`, not just the last one. That's one unified
   validity mechanism (joint space, always - the only space where limits/
   collisions are physically real) reused everywhere, with zero new planner
   instances and zero change to how Cartesian paths are geometrically
   planned.
