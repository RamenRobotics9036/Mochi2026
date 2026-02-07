# Smooth Drive Implementation Plan

## Target File

**New Class:** `src/main/java/frc/robot/visutils/DriveSmooth.java`

This class encapsulates all joystick processing for smooth driving:
- Deadband with rescale
- Response curve shaping
- Slew-rate limiting

**Usage in RobotContainer:**
```java
private final DriveSmooth m_driveSmooth = new DriveSmooth();

// In configureBindings():
double leftX = m_driveSmooth.processTranslationX(-joystick.getLeftY());
double leftY = m_driveSmooth.processTranslationY(-joystick.getLeftX());
double rightX = m_driveSmooth.processRotation(-joystick.getRightX());
```

---

## Current State Analysis

**File:** `src/main/java/frc/robot/RobotContainer.java`

### What's Currently Implemented:
- **Deadband**: 10% via `MathUtil.applyDeadband(-joystick.getLeftY(), 0.1)` — but no rescale after deadband
- **Response Curve**: None — linear scaling only
- **Slew-rate Limiting**: None — instant response to joystick changes
- **Swerve Optimization**: Using `DriveRequestType.Velocity` but SwerveRequest deadbands are effectively disabled (0.0001)

### Current Flow:
```
Raw Joystick → Deadband (10%) → Scale by TeleoperatedSpeed → Apply to SwerveRequest
```

---

## Stage 1: Deadband with Rescale

**Goal:** Remove stick drift AND maintain full range of output (0.0 to 1.0) after deadband.

**Problem with current:** After applying 10% deadband, the output range is 0.0 to 0.9, not 0.0 to 1.0. This means you lose 10% of your max speed.

### Implementation:
1. Create `DriveSmooth.java` in `visutils/` package with helper method:
```java
package frc.robot.visutils;

import frc.robot.Constants.DriveConstants;

/**
 * Processes joystick inputs for smooth driving.
 * Applies deadband, response curve, and slew-rate limiting.
 */
public class DriveSmooth {

    /**
     * Apply deadband and rescale to maintain full output range.
     * @param value Raw joystick input (-1 to 1)
     * @param deadband Deadband threshold (e.g., 0.1 for 10%)
     * @return Processed value with deadband applied and rescaled to full range
     */
    private double applyDeadbandWithRescale(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        // Rescale: map [deadband, 1.0] to [0.0, 1.0]
        return Math.signum(value) * (Math.abs(value) - deadband) / (1.0 - deadband);
    }
}
```

2. Update `RobotContainer.java` to use `DriveSmooth` instance

3. Add constant to `Constants.java`:
```java
public static final class DriveConstants {
    public static final double kJoystickDeadband = 0.1;  // 10%
}
```

### Testing:
- Verify no movement when stick is centered
- Verify full speed when stick is at edge
- Print raw vs processed values to console

---

## Stage 2: Response Curve Shaping

**Goal:** Small stick movements produce gentle motion; full stick reaches full speed.

### Implementation Options:

**Option A: Squared Input (simple)**
```java
private double applyResponseCurve(double input) {
    return Math.signum(input) * input * input;  // Preserves sign
}
```

**Option B: Cubed Input (more aggressive)**
```java
private double applyResponseCurve(double input) {
    return input * input * input;  // Naturally preserves sign
}
```

**Option C: Configurable Exponential (most flexible)**
```java
private double applyResponseCurve(double input, double exponent) {
    return Math.signum(input) * Math.pow(Math.abs(input), exponent);
}
// exponent = 1.0 → linear
// exponent = 2.0 → squared
// exponent = 3.0 → cubed
```

### Recommended: Option C with constants

Add to `DriveSmooth.java`:
```java
/**
 * Applies power-based response curve for fine control at low speeds.
 * @param input Processed input (-1 to 1)
 * @param exponent Response curve exponent (1.0=linear, 2.0=squared, 3.0=cubed)
 * @return Curved value
 */
private double applyResponseCurve(double input, double exponent) {
    return Math.signum(input) * Math.pow(Math.abs(input), exponent);
}
```

Add to `Constants.java`:
```java
public static final class DriveConstants {
    public static final double kJoystickDeadband = 0.1;
    public static final double kTranslationExponent = 2.0;  // Squared for translation
    public static final double kRotationExponent = 2.0;     // Squared for rotation
}
```

### Updated Flow:
```
Raw Joystick → DriveSmooth.process*() → [Deadband+Rescale → Response Curve] → Scale by Speed → SwerveRequest
```

### Testing:
- Verify gentle motion at low stick deflection
- Verify full speed still achievable at full deflection
- Test driving in straight lines and turning

---

## Stage 3: Slew-rate Limiting (Acceleration Limiting)

**Goal:** Limit how fast commanded speed can change per cycle to remove "jerk."

### Implementation:
1. Add WPILib's `SlewRateLimiter` to `DriveSmooth.java`:
```java
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveSmooth {
    // Slew rate limiters for smooth acceleration
    private final SlewRateLimiter m_xLimiter;
    private final SlewRateLimiter m_yLimiter;
    private final SlewRateLimiter m_rotLimiter;

    public DriveSmooth() {
        m_xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        m_yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationSlewRate);
    }
}
```

2. Apply in public process methods:
```java
public double processTranslationX(double rawInput) {
    double deadbanded = applyDeadbandWithRescale(rawInput, DriveConstants.kJoystickDeadband);
    double curved = applyResponseCurve(deadbanded, DriveConstants.kTranslationExponent);
    return m_xLimiter.calculate(curved);
}
```

3. Add constants to `Constants.java`:
```java
public static final class DriveConstants {
    // ... existing constants ...
    public static final double kTranslationSlewRate = 3.0;  // m/s per second
    public static final double kRotationSlewRate = 3.0;     // rad/s per second
}
```

### Tuning Guidelines:
- **Higher value** (e.g., 5.0): Faster acceleration, more responsive, but more jerky
- **Lower value** (e.g., 2.0): Smoother acceleration, but sluggish feel
- Start at 3.0 and adjust based on driver preference

### Reset Consideration:
Slew limiters maintain state. Consider resetting when:
- Robot is disabled (optional)
- Switching between auto and teleop

Call from `RobotContainer`:
```java
m_driveSmooth.reset();
```

### Testing:
- Quick stick movements should accelerate smoothly
- Release stick should decelerate smoothly
- No "jerk" on direction changes

---

## Stage 4: Swerve Module Optimization

**Goal:** Prevent module twitching at low speeds and optimize wheel angles.

### 4A: Increase SwerveRequest Deadband

Current code has effectively disabled deadbands (0.0001). Increase to meaningful values:

```java
private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0.1 * TeleoperatedSpeed)      // 10% of max speed
        .withRotationalDeadband(0.1 * MaxAngularRate)  // 10% of max rotation
        .withDriveRequestType(DriveRequestType.Velocity);
```

This tells CTRE's swerve to not move modules when requested speeds are very low.

### 4B: Module State Optimization

CTRE's Phoenix 6 swerve already does optimize module states internally. Verify it's enabled in `CommandSwerveDrivetrain` or `TunerConstants`.

### 4C: Lock Wheel Angles at Zero Speed (Advanced)

If modules still twitch at standstill, implement wheel angle locking:

```java
// Track last commanded angle when speed was non-zero
private Rotation2d m_lastHeading = new Rotation2d();

// In default command, before applying request:
double speed = Math.hypot(leftX, leftY);
if (speed > 0.05) {  // If moving
    m_lastHeading = new Rotation2d(Math.atan2(leftY, leftX));
}

// When stopped, you could optionally point wheels to m_lastHeading
// or just let the increased deadband handle it
```

### 4D: Idle Behavior

Already implemented:
```java
RobotModeTriggers.disabled().whileTrue(
    drivetrain.applyRequest(() -> idle).ignoringDisable(true)
);
```

Consider adding idle state during teleop when stick is in deadband.

---

## Implementation Order

| Stage | Description | Difficulty | Impact |
|-------|-------------|------------|--------|
| 1 | Deadband with Rescale | Easy | Low |
| 2 | Response Curve | Easy | Medium |
| 3 | Slew-rate Limiting | Medium | **High** |
| 4 | Swerve Optimization | Medium | Medium |

**Recommended approach:** Implement stages 1-3 together, then tune. Stage 4 addresses edge cases.

---

## Final Code Structure

### DriveSmooth.java (complete)
```java
package frc.robot.visutils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;

/**
 * Utility class for processing joystick inputs with smooth driving curves.
 * Applies deadband, rescaling, response curves, and slew rate limiting.
 */
public class DriveSmooth {
    private final SlewRateLimiter m_xLimiter;
    private final SlewRateLimiter m_yLimiter;
    private final SlewRateLimiter m_rotLimiter;

    public DriveSmooth() {
        m_xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        m_yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationSlewRate);
    }

    /**
     * Applies deadband with linear rescaling to preserve full output range.
     */
    private double applyDeadbandWithRescale(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return Math.signum(value) * ((Math.abs(value) - deadband) / (1.0 - deadband));
    }

    /**
     * Applies power-based response curve for fine control at low speeds.
     */
    private double applyResponseCurve(double value, double exponent) {
        return Math.signum(value) * Math.pow(Math.abs(value), exponent);
    }

    /**
     * Processes translation X input (forward/backward).
     */
    public double processTranslationX(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kTranslationExponent);
        return m_xLimiter.calculate(curved);
    }

    /**
     * Processes translation Y input (left/right strafe).
     */
    public double processTranslationY(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kTranslationExponent);
        return m_yLimiter.calculate(curved);
    }

    /**
     * Processes rotation input.
     */
    public double processRotation(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kRotationExponent);
        return m_rotLimiter.calculate(curved);
    }

    /**
     * Resets all slew rate limiters. Call when transitioning from autonomous to teleop.
     */
    public void reset() {
        m_xLimiter.reset(0);
        m_yLimiter.reset(0);
        m_rotLimiter.reset(0);
    }
}
```

### RobotContainer.java usage
```java
// Field declaration
private final DriveSmooth m_driveSmooth = new DriveSmooth();

// In configureBindings():
m_drivetrain.setDefaultCommand(
    m_drivetrain.applyRequest(() ->
        drive.withVelocityX(m_driveSmooth.processTranslationX(-joystick.getLeftY()) * TeleoperatedSpeed)
             .withVelocityY(m_driveSmooth.processTranslationY(-joystick.getLeftX()) * TeleoperatedSpeed)
             .withRotationalRate(m_driveSmooth.processRotation(-joystick.getRightX()) * MaxAngularRate)
    )
);
```

### Constants.java DriveConstants class
```java
public static final class DriveConstants {
    public static final double kJoystickDeadband = 0.1;       // 10%
    public static final double kTranslationExponent = 2.0;    // Squared
    public static final double kRotationExponent = 2.0;       // Squared
    public static final double kTranslationSlewRate = 3.0;    // m/s per second
    public static final double kRotationSlewRate = 3.0;       // rad/s per second
}
```

---

## Testing Checklist

- [ ] Stage 1: No drift at center, full speed at edge
- [ ] Stage 2: Gentle at low deflection, full at max
- [ ] Stage 3: Smooth acceleration/deceleration
- [ ] Stage 4: No module twitching at standstill
- [ ] Overall: Driver comfort and control feel
- [ ] Edge cases: Quick direction reversal, diagonal driving
