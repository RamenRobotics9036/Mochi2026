# Vision Correction Test Plan

Tests use `injectDriftToGroundTruth` to physically displace the simulated robot while leaving
odometry untouched. Vision (cameras tied to ground truth) should detect the error and correct
the pose estimate back toward ground truth over time.

---

## Case 1 — Translation drift only, injected twice

Run a PathPlanner path in simulation with ground truth enabled.

- At **path start**: nudge the physical robot **12 inches to the right**.
- At **path midpoint**: nudge the physical robot **12 inches to the right** again.
- Rotation is **not** disturbed at any point.

**Expected**: Vision detects the translation error after each nudge and converges the pose
estimate back toward ground truth before the path ends.

---

## Case 2 — Translation drift + rotation drift, injected at start and mid

Run a PathPlanner path in simulation with ground truth enabled.

- At **path start**:
  - Nudge the physical robot **12 inches to the right**.
  - Rotate the physical robot **20 degrees** (ground truth heading offset).
- At **path midpoint**: nudge the physical robot **12 inches to the right** again (no additional rotation).
- Odometry and pose estimator are untouched throughout.

**Expected**: Vision corrects both the translational and rotational error introduced at the
start, and then handles the second translation nudge mid-path.

---

<!-- Add more cases below -->
