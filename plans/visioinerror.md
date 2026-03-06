# Plan: Add `getVisionErrorAtSnapTime()` to `CamOdometryInterface`

## Goal

Expose the result of `calcVisionErrorAtSnapTime()` — already implemented in `SingleCamOdometry` — through the `CamOdometryInterface` so callers can read how far vision disagrees with odometry at the moment the image was captured.

Returns `Optional<Transform2d>`: the transform from the drivetrain's sampled pose at snap-time to the vision-reported pose.  Empty if no sampler is set, the pose buffer has no entry for that timestamp, or no camera had a lock this cycle.

---

## Files to change

### 1. `CamOdometryInterface.java`

- Add import: `edu.wpi.first.math.geometry.Transform2d`
- Add method signature:

```java
/**
 * Returns the offset between where wheel odometry thought the robot was at
 * the image-capture timestamp and where vision says it is.
 * Empty if no pose sampler is wired up, the pose buffer is too short,
 * or no camera had a lock this cycle.
 */
Optional<Transform2d> getVisionErrorAtSnapTime();
```

---

### 2. `SingleCamOdometry.java`

**a) New field** (alongside `m_latestVisPose`):

```java
private Optional<Transform2d> m_latestVisionError = Optional.empty();
```

**b) Reset in `clearResults()`**:

```java
m_latestVisionError = Optional.empty();
```

**c) Compute in `addVisionMeasurementV1()`** — just before `m_estConsumer.accept(...)`, replacing the `$TODO` comment:

```java
m_latestVisionError = calcVisionErrorAtSnapTime(mt1.pose, mt1.timestampSeconds);

if (m_estConsumer != null) {
    m_estConsumer.accept(mt1.pose, mt1.timestampSeconds, m_curStdDevs);
}
```

**d) Implement the interface method**:

```java
@Override
public Optional<Transform2d> getVisionErrorAtSnapTime() {
    return m_latestVisionError;
}
```

Note: `calcVisionErrorAtSnapTime()` is now called, so any `@SuppressWarnings("unused")` on it can be removed (none currently present).

---

### 3. `MultiCamOdometry.java`

- Add import: `edu.wpi.first.math.geometry.Transform2d`
- Implement the interface method, delegating to the best-locked camera (same pattern as `getEstimatedPose()`):

```java
@Override
public Optional<Transform2d> getVisionErrorAtSnapTime() {
    if (m_perCycleState.bestLockedCam.isPresent()) {
        return m_perCycleState.bestLockedCam.get().getVisionErrorAtSnapTime();
    }
    return Optional.empty();
}
```

---

## No other files need changes

- `MultiCamOdometryFactory.java` — no changes; wiring is already done (`setPoseSampler` is called for each cam)
- `RobotContainer.java` — no changes; callers can consume `getVisionErrorAtSnapTime()` via the existing `m_multiCamlimelight` reference
