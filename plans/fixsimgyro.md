# Sim Gyro Analysis and Plan

## Goal

Make the simulated `Pigeon2` behave like a real sensor instead of a perfect mirror of CTRE's internal sim pose.

That should let us:

1. Inject **gyro-specific faults** like bias, drift, and noise into the drivetrain's pose estimator.
2. Drive the gyro from **ground-truth physical heading** so simulation matches the real-world separation between:
   - physical robot pose
   - sensor measurements
   - estimated pose

---

## Current State

### What CTRE is doing today

`CommandSwerveDrivetrain.startSimThread()` calls `updateSimState(dt, battery)` every 4 ms.

Phoenix's internal `SimSwerveDrivetrain` then:

- updates drive motor sims
- updates steer motor sims
- updates encoders
- computes chassis omega from module states
- integrates an internal `m_lastAngle`
- writes that angle directly into `Pigeon2SimState`

In other words, the sim gyro is currently:

- perfect
- derived from ideal module motion
- owned entirely by CTRE's private sim class

That means the simulated gyro is **not** a sensor model. It is just CTRE's perfect simulated heading.

### What our code is doing today

We also have a separate `GroundTruthSim` that tracks the robot's physical pose independently of odometry.

That is already useful for vision, because cameras are fed from ground truth instead of estimated pose.

But the gyro is still not sourced from that physical state.

So right now:

- **vision** sees ground truth
- **gyro** sees CTRE's perfect internal sim heading
- **pose estimator** fuses module odometry with that perfect gyro

This is why the sim does not really model bad gyro behavior, and why physical-vs-estimated heading is still coupled.

### One more mismatch in our code

`SingleCamOdometry` currently pushes `driveState.Pose.getRotation()` into MegaTag2 orientation updates.

That means the vision stack is using **estimated pose rotation**, not raw gyro heading.

Even after fixing the sim gyro, that should still be changed to:

- `driveState.RawHeading`, or
- directly from `Pigeon2`

Otherwise MT2 can still be fed estimator rotation and hide the separation we want.

---

## Why a small overwrite hack is not the right long-term fix

A tempting approach is:

1. let CTRE run `updateSimState()`
2. immediately overwrite `Pigeon2SimState` yaw with ground truth or a biased version

That can work as a quick experiment, but it has a structural problem:

- CTRE's private sim class keeps its own internal `m_lastAngle`
- on the next tick, CTRE will integrate from **its** angle again
- so we are permanently fighting the vendor sim instead of owning the gyro model

This gets especially messy when we want:

- constant gyro bias
- random-walk drift
- gyro lag
- physical heading from ground truth while CTRE's internal heading disagrees

So the real solution is not "patch the gyro after CTRE sim runs."

The real solution is:

**separate module/encoder simulation from gyro simulation**

---

## Recommended Architecture

### 1. Create a dedicated simulated gyro model first

Add a new class, something like:

```java
SimGyroModel
```

Suggested responsibilities:

- maintain simulated measured yaw
- maintain simulated measured yaw rate
- provide reset and fault injection APIs
- write outputs into `Pigeon2SimState`

Suggested API:

```java
public interface SimGyroModel {
    void reset();
    void update(double dtSeconds);
    Rotation2d getMeasuredHeading();
    double getMeasuredOmegaRadPerSec();
}
```

For the very first integration step, the implementation should be intentionally dumb:

- `ConstantAngleGyroModel`
- measured yaw always returns `45°`
- measured omega always returns `0`

This is useful because it isolates the integration problem:

- can we replace CTRE's simulated gyro source at all?
- does `RawHeading` become `45°`?
- does field-centric driving react to the simulated gyro reading?
- does vision consume the new simulated gyro reading?

Only after this works should we make the gyro model smarter.

### 2. Make ground truth the authoritative physical state

`GroundTruthSim` should become the source of physical robot motion in sim.

Instead of only updating in `Robot.simulationPeriodic()` with wall-clock deltas, it should support:

```java
void update(double dtSeconds)
```

That lets it run at the same 4 ms cadence as the drivetrain sim loop.

Why this matters:

- gyro sensors should update at the fast sim rate
- if ground truth only advances every 20 ms, gyro readings become stair-stepped and stale

### 3. Evolve the gyro model from constant-angle to physical-angle

Once the integration path is proven with the constant `45°` gyro, upgrade the model so it is driven from physical heading and angular velocity.

Suggested future API:

```java
public interface SimGyroModel {
    void reset(Rotation2d physicalHeading);
    void update(double dtSeconds, Rotation2d physicalHeading, double physicalOmegaRadPerSec);
    Rotation2d getMeasuredHeading();
    double getMeasuredOmegaRadPerSec();
}
```

Start with a deterministic implementation first:

- `ConstantAngleGyroModel`
- measured yaw = `45°`
- measured omega = `0`

Then the next deterministic implementation becomes:

- `PerfectGroundTruthGyroModel`
- measured yaw = physical yaw
- measured omega = physical omega

Then extend with:

- fixed yaw offset
- bias drift over time
- noise

### 4. Stop letting CTRE own gyro simulation

This is the most important design decision.

We should **not** rely on Phoenix's built-in `SimSwerveDrivetrain` to simulate the Pigeon if we want custom gyro behavior.

Instead, create a local copy/adaptation of the vendor sim class, for example:

```java
frc.robot.sim.ctre.CustomSimSwerveDrivetrain
```

This class should:

- keep the useful CTRE logic for drive/steer motor simulation
- keep encoder updates
- **remove gyro integration ownership**

So it updates:

- drive motors
- steer motors
- encoders

But **does not** set Pigeon yaw itself.

Then our own `SimGyroModel` becomes the only writer to `Pigeon2SimState`.

This is the cleanest way to get real sensor modeling without fighting private vendor state.

### 5. Run all physical sim in one coordinated 4 ms loop

The 4 ms sim loop in `CommandSwerveDrivetrain` should become the coordinator for:

1. module/encoder hardware sim
2. ground-truth physics update
3. gyro model update
4. writing gyro values to `Pigeon2SimState`

Suggested order:

1. read current drivetrain speeds
2. update custom hardware sim for motors/encoders
3. update `GroundTruthSim` using the same `dtSeconds`
4. update `SimGyroModel` from ground truth
5. write measured yaw / yaw rate into Pigeon sim state

This gives us:

- one physical truth
- one sensor model
- one estimator consuming that sensor

which is much closer to the real robot stack.

---

## What this unlocks

### A. Ground-truth-driven gyro

If physical robot heading changes in ground truth, the gyro changes too.

That means:

- rotating the physical robot in sim changes the gyro immediately
- the drivetrain estimator only knows what the gyro told it
- estimated heading can now differ from physical heading in realistic ways

### B. Bad gyro testing

Because the drivetrain estimator already consumes the Pigeon, any gyro fault we inject at the sim sensor layer will naturally flow into CTRE odometry/Kalman fusion.

This is exactly what we want for tests like:

- fixed +5 degree heading bias
- slow heading drift over a 15 second auto
- noisy gyro causing slight estimator wobble

### C. More realistic fault boundaries

With this architecture:

- `injectDriftToGroundTruth(...)` changes physical state
- gyro follows physical state
- estimator may or may not catch up depending on sensor faults and vision

And:

- `injectDriftToPoseEstimate(...)` becomes a pure estimator fault injection tool

One important consequence:

If you inject only estimator rotation error while the gyro is healthy, the estimator may snap back quickly.

That is not a bug. That is realistic.

If we want heading-estimation failures, those should come from the gyro model, not from forcing fake pose rotation offsets.

---

## Concrete Implementation Plan

## Step 1 - Add a minimal simulated gyro that always reports 45 degrees

Create new files:

- `src/main/java/frc/robot/sim/gyromodel/SimGyroModel.java`
- `src/main/java/frc/robot/sim/gyromodel/ConstantAngleGyroModel.java`

Initial behavior:

- measured yaw = `Rotation2d.fromDegrees(45)`
- measured omega = `0.0`
- deterministic, no noise, no ground-truth dependency yet

Use this step only to prove integration:

- we can drive `Pigeon2SimState` from our own gyro model
- `getState().RawHeading` reflects the custom gyro model
- field-centric / heading-dependent code now uses the new simulated gyro path

Add future-ready hooks if convenient, but they are optional in this first step.

## Step 2 - Integrate the custom gyro into the drivetrain sim path

In `CommandSwerveDrivetrain` and sim wiring:

- instantiate the new gyro model in simulation
- update it every sim tick
- write its values into `getPigeon2().getSimState()`

At this stage, the gyro should still always read `45°`.

This step is complete when the robot behaves as if the gyro is stuck at `45°`.

## Step 3 - Copy/adapt CTRE sim so gyro is no longer vendor-owned

Create a local sim helper based on Phoenix's `SimSwerveDrivetrain`:

- `src/main/java/frc/robot/sim/ctre/CustomSimSwerveDrivetrain.java`

It should:

- simulate drive motors
- simulate steer motors
- simulate encoders
- not integrate `m_lastAngle`
- not write yaw into the Pigeon sim

This is the key step that makes the rest clean.

## Step 4 - Wire the full custom sim loop in `CommandSwerveDrivetrain`

In `CommandSwerveDrivetrain`:

- replace the current `updateSimState(...)` usage in the sim thread
- call the custom hardware sim instead
- update the custom gyro model
- write the gyro model output into `getPigeon2().getSimState()`

At this stage, the gyro model should still be the constant-`45°` implementation.

The sim thread becomes the place where the simulated hardware and the simulated gyro advance together.

## Step 5 - Let `SimWrapper` own the composition

`SimWrapper` should create and wire:

- `SimGyroModel`
- later, `GroundTruthSim`
- any fault controls for the gyro

That keeps sim-specific composition out of `RobotContainer`.

Good additions to `FaultyAutoSim` later:

- `setGyroYawOffsetDegrees(...)`
- `enableGyroDrift(...)`
- `resetGyroFaults()`

## Step 6 - Add focused tests for the constant-angle gyro integration

Add tests for these behaviors first:

### Constant-angle integration tests

- simulated Pigeon yaw reports `45°`
- `RawHeading` reports `45°`
- field-centric heading logic behaves as if the gyro is fixed at `45°`
- drivetrain heading-dependent logic is consuming the custom gyro path

This phase should be considered done only after the robot stack is clearly running from the fake gyro instead of Phoenix's built-in Pigeon sim.

## Step 7 - Make ground truth update by explicit dt

Refactor `GroundTruthSim`:

- keep `simulationPeriodic()` for telemetry/publishing
- add `update(double dtSeconds)`
- stop depending on `Utils.getCurrentTimeSeconds()` for the core integration path

Also add:

- `Rotation2d getGroundTruthHeading()`
- `double getGroundTruthOmegaRadPerSec()`

The omega can be:

- stored from the most recent update
- derived from the same motion/fault model used for heading integration

## Step 8 - Replace the constant-angle gyro with ground-truth-driven gyro

Upgrade the gyro model implementation:

- replace `ConstantAngleGyroModel` with `PerfectGroundTruthGyroModel`
- feed it physical heading and physical omega from `GroundTruthSim`

Success criteria:

- physical heading changes cause gyro changes
- estimated pose heading can now diverge from physical heading
- the estimator only knows what the gyro told it

## Step 9 - Change vision heading consumers to use raw gyro heading

Update `SingleCamOdometry` so MegaTag2 orientation uses:

- `driveState.RawHeading`

not:

- `driveState.Pose.getRotation()`

This is intentionally late in the plan now. The idea is to first prove the drivetrain stack itself is running from the fake gyro, and only after that update vision consumers to match.

## Step 10 - Add focused tests for the ground-truth and vision-integrated version

Add tests for these behaviors:

### Ground-truth gyro tests

- when physical heading changes, simulated Pigeon yaw changes
- when physical heading stays fixed, simulated Pigeon yaw stays fixed

### Gyro fault tests

- fixed offset produces corresponding heading error in raw gyro
- drift accumulates over time

### Estimator separation tests

- ground truth heading can differ from estimated heading
- gyro bias affects estimator heading
- vision can correct translation while heading still reflects gyro fault behavior

### Vision consumer tests

- MT2 orientation push uses `RawHeading`, not `Pose.getRotation()`

---

## Recommended First Increment

To keep risk manageable, do this in two phases.

### Phase 1 - Correct architecture boundary

Implement:

- `SimGyroModel`
- constant `45°` gyro

But keep the gyro intentionally dumb at first.

Success criteria:

- simulated gyro is no longer vendor-owned
- the robot stack responds to a custom gyro value

### Phase 2 - Add gyro faults

After the constant-angle path is stable, replace it with a ground-truth-driven gyro, then update the remaining consumers, then add:

- perfect ground-truth heading
- MT2 uses raw heading
- yaw offset
- drift
- optional noise

This keeps debugging simpler because we first prove the separation works before adding fault complexity.

---

## Recommendation

The best solution is:

**treat the gyro as its own simulated sensor model, driven by ground truth, and stop letting CTRE's built-in sim own the Pigeon heading.**

But the best first implementation step is even smaller:

**first prove we can inject a totally fake gyro, like a constant `45°` reading, all the way through the stack.**

Once that wiring works, replacing the constant source with ground truth becomes much safer and easier to debug.

That is the only approach that cleanly supports both of your goals:

1. realistic separation between physical pose and estimated pose
2. deliberate gyro inaccuracy injected into the drivetrain estimator

Anything smaller than that is likely to turn into a fragile overwrite loop against private Phoenix sim state.
