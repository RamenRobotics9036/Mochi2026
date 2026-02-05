# Vision-Only Position Averaging for Motionless Robot

## Goal
Create an accurate position estimate using **only vision measurements** when the robot is stationary, accepting measurements only when >1 AprilTag is visible (MegaTag accuracy).

## Assumptions
- Robot is **motionless** during measurement collection
- Only measurements with **≥2 visible targets** are accepted
- We want the estimate to "settle" over time to a highly accurate value
- No odometry or gyro inputs

---

## Selected Approach: Vision-Only Kalman Filter

A simplified Kalman filter where:
- **State**: `[x, y, θ]` (robot pose)
- **Process model**: Identity (assume no motion: `x_{k+1} = x_k`)
- **Measurement**: Vision pose from MegaTag
- **Process noise Q**: Very small (robot is stationary)
- **Measurement noise R**: Based on MegaTag confidence/tag count

**Why this approach:**
1. **Convergence behavior**: Covariance matrix P naturally shrinks as measurements arrive, giving a clear "settled" indicator
2. **Confidence weighting**: Can scale R by (1/tagCount) or (1/confidence) to trust better measurements more
3. **Outlier handling**: If a bad measurement arrives, high R means it has less effect
4. **Mathematical rigor**: Optimal estimator for Gaussian noise

---

## Design Decisions

| Decision | Choice | Notes |
|----------|--------|-------|
| **Location** | `frc/robot/visutils/VisionKalmanFilter.java` | New class in visutils package |
| **Reset trigger** | Manual | Call `reset()` explicitly when needed |
| **Motionless detection** | Gyro angular rate threshold | See recommendation below |
| **Output** | NetworkTables `BasicInfo/VisionKalmanPose` | Published via BasicInfoDashboard |

### Motionless Detection (Recommended Approach)

**Use Pigeon2 angular velocity** as the primary indicator:

```java
// In Constants.java
public static final double kMotionlessGyroThreshold = 2.0; // deg/sec

// Check motionless
boolean isMotionless = Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble()) < kMotionlessGyroThreshold;
```

**Why gyro over chassis velocity:**
- **More direct**: Gyro measures actual motion, not commanded/estimated motion
- **Lower latency**: Instant reading vs. derived from wheel encoders
- **Catches bumps/vibration**: Detects if robot is jostled even if wheels aren't moving
- **Simpler**: Single sensor check vs. computing velocity magnitude from ChassisSpeeds

**Alternative (can combine both):**
```java
ChassisSpeeds speeds = drivetrain.getState().Speeds;
double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
boolean isMotionless = linearSpeed < 0.05 && Math.abs(gyroRate) < 2.0;
```

---

## Implementation Plan

### Phase 1: Core Filter Class
**File:** `src/main/java/frc/robot/visutils/VisionKalmanFilter.java`

```java
public class VisionKalmanFilter {
    private Matrix<N3, N1> m_x;      // State: [x, y, θ]
    private Matrix<N3, N3> m_P;      // Covariance
    private static final Matrix<N3, N3> Q = ...;  // Process noise (very small)
    private boolean m_initialized = false;

    /** Inject a vision measurement. Ignored if tagCount < 2. */
    public void injectVisionMeasurement(Pose2d pose, int tagCount) {
        if (tagCount < 2) return;

        if (!m_initialized) {
            // First measurement initializes state
            m_x = new Matrix<>(Nat.N3(), Nat.N1());
            m_x.set(0, 0, pose.getX());
            m_x.set(1, 0, pose.getY());
            m_x.set(2, 0, pose.getRotation().getRadians());
            m_P = initialCovariance();
            m_initialized = true;
            return;
        }

        // Compute measurement noise R (lower for more tags)
        Matrix<N3, N3> R = computeMeasurementNoise(tagCount);

        // Kalman update (H = I for direct pose measurement)
        // K = P * (P + R)^-1
        // x = x + K * (z - x)
        // P = (I - K) * P
    }

    public Pose2d getEstimate() { ... }
    public Matrix<N3, N3> getCovariance() { return m_P; }
    public boolean isInitialized() { return m_initialized; }

    /** Check if position uncertainty is below threshold */
    public boolean hasConverged(double positionThresholdMeters, double angleThresholdRad) {
        if (!m_initialized) return false;
        return m_P.get(0, 0) < positionThresholdMeters * positionThresholdMeters
            && m_P.get(1, 1) < positionThresholdMeters * positionThresholdMeters
            && m_P.get(2, 2) < angleThresholdRad * angleThresholdRad;
    }

    public void reset() {
        m_initialized = false;
        m_x = null;
        m_P = null;
    }
}
```

### Phase 2: Integration with Vision Subsystem
1. Add `VisionKalmanFilter` instance to vision subsystem or RobotContainer
2. In vision update loop:
   ```java
   if (isMotionless && tagCount >= 2) {
       m_visionKalman.injectVisionMeasurement(visionPose, tagCount);
   }
   ```
3. Expose getters for estimate and convergence state

### Phase 3: Dashboard Publishing
**File:** `BasicInfoDashboard.java`

Add new publishers:
```java
// New fields
private final DoubleArrayPublisher m_visionKalmanPose =
    m_basicInfoTable.getDoubleArrayTopic("VisionKalmanPose").publish();
private final BooleanPublisher m_visionKalmanConverged =
    m_basicInfoTable.getBooleanTopic("VisionKalmanConverged").publish();
private final BooleanPublisher m_visionKalmanActive =
    m_basicInfoTable.getBooleanTopic("VisionKalmanActive").publish();

// Supplier
private Supplier<VisionKalmanFilter> m_visionKalmanSupplier = null;

public void setVisionKalmanSupplier(Supplier<VisionKalmanFilter> supplier) {
    m_visionKalmanSupplier = supplier;
}

// In update()
if (m_visionKalmanSupplier != null) {
    VisionKalmanFilter filter = m_visionKalmanSupplier.get();
    m_visionKalmanActive.set(filter.isInitialized());
    if (filter.isInitialized()) {
        Pose2d pose = filter.getEstimate();
        m_visionKalmanPose.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
        m_visionKalmanConverged.set(filter.hasConverged(0.02, Math.toRadians(1.0)));
    }
}
```

---

## Tuning Parameters

| Parameter | Suggested Value | Notes |
|-----------|-----------------|-------|
| Initial P (position) | 1.0 m² | High uncertainty at start |
| Initial P (angle) | 0.5 rad² | High uncertainty at start |
| Q (position) | 0.0001 m² | Very small - robot is stationary |
| Q (angle) | 0.00005 rad² | Very small |
| R base (position) | 0.05 m² | Base measurement noise |
| R base (angle) | 0.02 rad² | Base measurement noise |
| R scaling | R_base / tagCount | More tags = more trust |
| Convergence threshold (pos) | 0.02 m | ~2cm uncertainty |
| Convergence threshold (angle) | 1.0° | ~1 degree uncertainty |
| Motionless gyro threshold | 2.0 deg/s | Below this = stationary |

---

## Task Checklist

- [x] Create `VisionKalmanFilter` class in `visutils/`
- [x] Add constants to `Constants.java` (thresholds, noise values)
- [x] Integrate with vision subsystem (inject measurements)
- [x] Add motionless detection using Pigeon2 gyro rate
- [x] Add `reset()` trigger (POV Down button in simulation)
- [x] Add NetworkTables publishers to `BasicInfoDashboard`
- [x] Wire up supplier in `RobotContainer`
- [ ] Test and tune parameters
