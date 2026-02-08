# Motionless Tracking Refactor Plan

## Goal
Extract motionless-tracking logic from `RobotContainer` into a new standalone class `visutils/MotionlessTracker.java` to reduce complexity in `RobotContainer`.

---

## Current State

All motionless logic lives in **RobotContainer.java** (lines 94-196):

| Item | Location | Purpose |
|------|----------|---------|
| `m_wasStillLastCycle` | field (L94) | Edge-detection: was still last cycle? |
| `m_stillStartTime` | field (L96) | FPGA timestamp when stillness began |
| `m_isCurrentlyStill` | field (L98) | Cached current-cycle result |
| `updateMotionlessTracking()` | method (L151) | Periodic update — detects edges, resets Kalman filter |
| `isRobotMotionless()` | method (L171) | Returns cached `m_isCurrentlyStill` |
| `computeIsMotionless()` | method (L178) | Reads drivetrain speeds, compares to thresholds from `VisionKalmanConstants` |
| `getSecondsStill()` | method (L192) | Returns seconds since robot became still |

### Consumers
- **Robot.java L62** — calls `m_robotContainer.updateMotionlessTracking()` every cycle.
- **RobotContainer constructor** — wires `this::isRobotMotionless` and `this::getSecondsStill` as suppliers to `LimelightOdometry` and `BasicInfoDashboard`.
- **LimelightOdometry** — uses `BooleanSupplier` for `isMotionless` to gate Kalman filter updates.

### Constants
- `VisionKalmanConstants.kMotionlessLinearThreshold` (0.05 m/s)
- `VisionKalmanConstants.kMotionlessGyroThreshold` (2.0 deg/s)

---

## Step-by-Step Refactor

### Step 1 — Create `visutils/MotionlessTracker.java`

New class with the following API:

```java
package frc.robot.visutils;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionKalmanConstants;

public class MotionlessTracker {
    private boolean m_wasStillLastCycle = false;
    private double  m_stillStartTime   = 0.0;
    private boolean m_isCurrentlyStill = false;

    private final Supplier<ChassisSpeeds> m_speedsSupplier;
    private Runnable m_onStartedMoving = null;   // callback for Kalman reset

    public MotionlessTracker(Supplier<ChassisSpeeds> speedsSupplier) { ... }

    /** Optional callback fired on the still→moving edge. */
    public void setOnStartedMoving(Runnable callback) { ... }

    /** Call once per robot periodic cycle. */
    public void update() { ... }

    /** True when the robot is stationary this cycle. */
    public boolean isMotionless() { ... }

    /** Seconds the robot has been continuously still (0 if moving). */
    public double getSecondsStill() { ... }
}
```

Key design choices:
- Takes a `Supplier<ChassisSpeeds>` so it has no direct dependency on the drivetrain.
- Uses a `Runnable` callback (`m_onStartedMoving`) to notify when the robot starts moving, so `RobotContainer` can use it to reset the Kalman filter without the tracker needing to know about `VisionKalmanFilter`.
- Thresholds still come from `Constants.VisionKalmanConstants` (single source of truth).

### Step 2 — Instantiate in `RobotContainer`

Add the new tracker as a field:

```java
private final MotionlessTracker m_motionlessTracker =
    new MotionlessTracker(() -> drivetrain.getState().Speeds);
```

In the constructor, wire the Kalman-reset callback:

```java
m_motionlessTracker.setOnStartedMoving(m_visionKalmanFilter::reset);
```

### Step 3 — Replace supplier references in `RobotContainer` constructor

| Old | New |
|-----|-----|
| `this::isRobotMotionless` | `m_motionlessTracker::isMotionless` |
| `this::getSecondsStill` | `m_motionlessTracker::getSecondsStill` |

Affected lines:
- `m_limelightOdometry.setVisionKalmanFilter(...)` — second arg becomes `m_motionlessTracker::isMotionless`
- `basicInfoDashboard.setIsRobotMotionlessSupplier(...)` — same
- `basicInfoDashboard.setSecondsStillSupplier(...)` — becomes `m_motionlessTracker::getSecondsStill`

### Step 4 — Update `Robot.java`

Change:
```java
m_robotContainer.updateMotionlessTracking();
```
to:
```java
m_robotContainer.m_motionlessTracker.update();
```
(or keep the existing call and have `updateMotionlessTracking()` delegate — either works, but direct access is simpler since `m_motionlessTracker` can be `public final`.)

### Step 5 — Delete old code from `RobotContainer`

Remove:
- Fields: `m_wasStillLastCycle`, `m_stillStartTime`, `m_isCurrentlyStill`
- Methods: `updateMotionlessTracking()`, `isRobotMotionless()`, `computeIsMotionless()`, `getSecondsStill()`

### Step 6 — Build & verify

```
./gradlew build
```

No logic changes — purely a move. All consumers now go through `MotionlessTracker`.

---

## Files Changed (summary)

| File | Action |
|------|--------|
| `visutils/MotionlessTracker.java` | **Create** — new class |
| `RobotContainer.java` | **Edit** — remove 6 members, add field + wiring |
| `Robot.java` | **Edit** — update one call site |

No changes needed in `LimelightOdometry`, `BasicInfoDashboard`, or `Constants` — they already consume via suppliers / constants.
