# Plan: VisionHeartBeat

## Overview

Add `src/main/java/frc/robot/visutils/VisionHeartBeat.java` — a lightweight diagnostic class that polls a single Limelight camera every N periodic cycles and exposes four boolean status flags.

---

## Class Location & Package

```
src/main/java/frc/robot/visutils/VisionHeartBeat.java
package frc.robot.visutils;
```

---

## Constructor

```java
public VisionHeartBeat(String limelightName)
public VisionHeartBeat(String limelightName, int checkIntervalCycles)
```

- `limelightName` — passed straight to `LimelightHelpers` calls.
- `checkIntervalCycles` — how many `update()` calls to skip between real checks. Default: `20` (≈400 ms at 50 Hz).

---

## State Fields

| Field | Type | Purpose |
|---|---|---|
| `m_limelightName` | `String` | camera name |
| `m_checkInterval` | `int` | cycles between real checks |
| `m_cycleCount` | `int` | counts calls to `update()` |
| `m_lastHeartbeat` | `double` | last seen heartbeat value |
| `m_isHeartbeating` | `boolean` | cached result — flag 1 |
| `m_hasTid` | `boolean` | cached result — flag 2 |
| `m_hasMt1Pose` | `boolean` | cached result — flag 3 |
| `m_hasMt2Pose` | `boolean` | cached result — flag 4 |

---

## update() Logic

Called once per periodic cycle. Only does real NetworkTables work every `m_checkInterval` calls.

```
cycleCount++
if (cycleCount % checkInterval != 0) return   // return early, cached values unchanged

// --- heartbeat ---
newHeartbeat = LimelightHelpers.getHeartbeat(name)
isHeartbeating = (newHeartbeat != lastHeartbeat)
lastHeartbeat  = newHeartbeat

// --- tid ---
hasTid = (LimelightHelpers.getFiducialID(name) != -1)

// --- MegaTag1 pose ---
mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name)
hasMt1Pose = LimelightHelpers.validPoseEstimate(mt1)

// --- MegaTag2 pose ---
mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
hasMt2Pose = LimelightHelpers.validPoseEstimate(mt2)
```

`LimelightHelpers.validPoseEstimate()` already null-checks and verifies `rawFiducials.length != 0`, so use it directly.

---

## Public API

```java
/** Call once per robot periodic. */
public void update()

/** True if the Limelight heartbeat counter is incrementing. */
public boolean isHeartbeating()

/** True if a fiducial target has been detected (tid != -1). */
public boolean hasTid()

/** True if MegaTag1 has returned a valid pose estimate. */
public boolean hasMt1Pose()

/** True if MegaTag2 has returned a valid pose estimate. */
public boolean hasMt2Pose()
```

---

## Notes

- The four cached booleans are initialized to `false`; they stay `false` until the first real check fires (after `checkInterval` cycles). This is intentional — callers should treat `false` as "not yet confirmed", not "definitely broken".
- No SmartDashboard/logging calls are included in this class itself; callers can log the four getters if desired.
- No dependency on `Constants.java` is needed unless the team later wants to centralise the default interval. For now, hard-code the default as `20` inside the class.
- The two-arg constructor exists so callers can tune polling frequency (e.g. slower checks while disabled, or faster during testing).

---

## Unit Tests

**File:** `src/test/java/frc/robot/visutils/TestVisionHeartBeat.java`

### Supplier-injection design

The production `(String limelightName)` constructor delegates to a package-private constructor that accepts four suppliers:

```java
VisionHeartBeat(
    DoubleSupplier heartbeatSupplier,
    DoubleSupplier tidSupplier,
    Supplier<LimelightHelpers.PoseEstimate> mt1Supplier,
    Supplier<LimelightHelpers.PoseEstimate> mt2Supplier,
    int checkIntervalCycles
)
```

Tests create the object using this constructor with plain lambdas — no NT, no HAL init required.

### Test setup

Use `checkIntervalCycles = 1` in all tests so every `update()` call triggers a real check (no skipping).

For a "valid" `PoseEstimate`, construct one with `tagCount > 0` and a non-empty `rawFiducials` array, since `validPoseEstimate()` checks `rawFiducials.length != 0`.

For an "invalid" estimate, use `new PoseEstimate()` (default constructor — empty `rawFiducials`).

```java
private static LimelightHelpers.PoseEstimate validEstimate() {
    LimelightHelpers.RawFiducial rf = new LimelightHelpers.RawFiducial(1, 0, 0, 0, 1.0, 1.0, 0.1);
    return new LimelightHelpers.PoseEstimate(
        new Pose2d(), 0, 0, 1, 0, 1.0, 0, new LimelightHelpers.RawFiducial[]{rf}, false);
}

private static LimelightHelpers.PoseEstimate invalidEstimate() {
    return new LimelightHelpers.PoseEstimate(); // rawFiducials = empty array
}
```

### Test cases

**Heartbeat**

- `heartbeat_notIncrementing_isFalse` — supplier always returns the same value; assert `isHeartbeating() == false` after two `update()` calls.
- `heartbeat_incrementing_isTrue` — supplier returns incrementing values (use `AtomicLong`); assert `isHeartbeating() == true`.
- `heartbeat_stopsIncrementing_becomesFalse` — starts incrementing (true), then freezes; assert flips to false after next check.

**Tid**

- `tid_negativeOne_isFalse` — supplier returns `-1.0`; assert `hasTid() == false`.
- `tid_validId_isTrue` — supplier returns `5.0`; assert `hasTid() == true`.

**MegaTag1 pose**

- `mt1_invalidEstimate_isFalse` — supplier returns `invalidEstimate()`; assert `hasMt1Pose() == false`.
- `mt1_validEstimate_isTrue` — supplier returns `validEstimate()`; assert `hasMt1Pose() == true`.

**MegaTag2 pose**

- `mt2_invalidEstimate_isFalse` / `mt2_validEstimate_isTrue` — same pattern as MT1.

**Interval skipping**

- `cachedValues_returnedBetweenChecks` — use `checkIntervalCycles = 3`. Start with heartbeat not incrementing (false). After 1st `update()` (cycle 1, triggers check) assert false. Switch to incrementing supplier. Call `update()` twice more (cycles 2 and 3, skipped). Assert still false (cached). Call `update()` again (cycle 4, triggers check). Assert now true.

**Initial state**

- `initialState_allFalse` — construct with `checkIntervalCycles = 5`. Without calling `update()`, assert all four getters return `false`.
