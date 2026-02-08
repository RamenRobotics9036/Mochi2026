package frc.robot.visutils;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionKalmanConstants;

/**
 * Tracks whether the robot is motionless based on chassis speeds.
 *
 * <p>Provides edge-detection (still→moving, moving→still) and a running
 * duration of how long the robot has been stationary. An optional callback
 * fires on the still→moving transition so callers can reset state (e.g.
 * resetting a Kalman filter).
 *
 * <p>Call {@link #update()} once per robot periodic cycle.
 */
public class MotionlessTracker {

    private final Supplier<ChassisSpeeds> m_speedsSupplier;

    /** Optional callback fired when the robot transitions from still to moving. */
    private Runnable m_onStartedMoving = null;

    /** Tracks whether robot was still on the previous cycle (for edge detection). */
    private boolean m_wasStillLastCycle = false;
    /** FPGA timestamp when robot became still (seconds). */
    private double m_stillStartTime = 0.0;
    /** Cached value of whether robot is currently still. */
    private boolean m_isCurrentlyStill = false;

    /**
     * Creates a new MotionlessTracker.
     *
     * @param speedsSupplier Supplier for the current chassis speeds (e.g.
     *                       {@code () -> drivetrain.getState().Speeds}).
     */
    public MotionlessTracker(Supplier<ChassisSpeeds> speedsSupplier) {
        m_speedsSupplier = speedsSupplier;
    }

    /**
     * Sets an optional callback that fires when the robot transitions from
     * still to moving (e.g. to reset a Kalman filter).
     *
     * @param callback Runnable to invoke on the still→moving edge, or null to clear.
     */
    public void setOnStartedMoving(Runnable callback) {
        m_onStartedMoving = callback;
    }

    /**
     * Updates the motionless state tracking. Call this once per robot periodic cycle.
     */
    public void update() {
        m_isCurrentlyStill = computeIsMotionless();

        if (m_isCurrentlyStill && !m_wasStillLastCycle) {
            // Robot just became still — record the time
            m_stillStartTime = Timer.getFPGATimestamp();
        } else if (!m_isCurrentlyStill && m_wasStillLastCycle) {
            // Robot just started moving — fire callback
            if (m_onStartedMoving != null) {
                m_onStartedMoving.run();
            }
        }

        m_wasStillLastCycle = m_isCurrentlyStill;
    }

    /**
     * Returns whether the robot is motionless this cycle.
     *
     * @return true if the robot is stationary
     */
    public boolean isMotionless() {
        return m_isCurrentlyStill;
    }

    /**
     * Gets how long the robot has been continuously still, in seconds.
     *
     * @return Seconds the robot has been motionless, or 0.0 if currently moving
     */
    public double getSecondsStill() {
        if (!m_isCurrentlyStill) {
            return 0.0;
        }
        return Timer.getFPGATimestamp() - m_stillStartTime;
    }

    /**
     * Computes whether the robot is motionless based on chassis speeds
     * and the thresholds in {@link VisionKalmanConstants}.
     */
    private boolean computeIsMotionless() {
        ChassisSpeeds speeds = m_speedsSupplier.get();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularRateDegPerSec = Math.abs(Math.toDegrees(speeds.omegaRadiansPerSecond));

        return linearSpeed < VisionKalmanConstants.kMotionlessLinearThreshold
            && angularRateDegPerSec < VisionKalmanConstants.kMotionlessGyroThreshold;
    }
}
