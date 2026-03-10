# Vision Improvement Plan

Tests use `injectDriftToGroundTruth` to physically displace the simulated robot while leaving
odometry untouched. Vision (cameras tied to ground truth) should detect the error and correct
the pose estimate back toward ground truth over time.

---

## Sim Test Cases

### Case 1 — Translation drift only, injected twice

Run a PathPlanner path in simulation with ground truth enabled.

- At **path start**: nudge the physical robot **12 inches to the right**.
- At **path midpoint**: nudge the physical robot **12 inches to the right** again.
- Rotation is **not** disturbed at any point.

**Expected**: Vision detects the translation error after each nudge and converges the pose
estimate back toward ground truth before the path ends.

---

### Case 2 — Translation drift + rotation drift, injected at start and mid

Run a PathPlanner path in simulation with ground truth enabled.

- At **path start**:
  - Nudge the physical robot **12 inches to the right**.
  - Rotate the physical robot **20 degrees** (ground truth heading offset).
- At **path midpoint**: nudge the physical robot **12 inches to the right** again (no additional rotation).
- Odometry and pose estimator are untouched throughout.

**Expected**: Vision corrects both the translational and rotational error introduced at the
start, and then handles the second translation nudge mid-path.

---

### Case 3 — Single camera with miscalibrated robot-to-camera transform

Run with only one camera active. The robot-to-camera transform reported to the vision system
is offset by **1 foot** from its true physical position on the robot.

**Expected**: The pose estimates are consistently biased by an amount corresponding to the
transform error. The bias is stable (not oscillating) and its magnitude/direction matches the
1-foot offset in the miscalibrated transform.

---

### Case 4 — Two cameras with conflicting robot-to-camera transforms

Run with two cameras active. One camera's reported position is correct; the other's reported
position is offset by **1 foot** from the robot center (miscalibrated transform).

**Expected**: The two cameras produce conflicting pose measurements. Observe how the pose
estimator handles the disagreement — ideally the well-calibrated camera's measurements
dominate, and the injected bias from the miscalibrated camera is visible in the per-camera
error diagnostics.

---

<!-- Add more cases below -->

## Diagnostic Changes Needed

- **Per-camera error arrows on robot** — For each camera, display the XY error between the
  vision pose snapshot and the robot's odometry position *at the time of that snapshot* as an
  arrow overlaid on the robot graphic. Arrow length encodes magnitude; arrow direction encodes
  direction of the discrepancy.

- **Vision injection rate display** — Show the number of vision pose injections fed into the
  drivetrain per second, computed as a per-second average over a 5-second rolling window.
