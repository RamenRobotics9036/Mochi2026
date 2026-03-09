package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * Wrapper around a real {@link CamOdometryInterface} that supports enable/disable.
 *
 * <p>When enabled, all calls are delegated to {@code m_multiCamReal}. When disabled, all calls are
 * delegated to {@code m_multiCamNoop}, which returns safe default values and performs no side
 * effects.
 */
public class MultiCamOdometryWrapper implements CamOdometryInterface {
    private final CamOdometryInterface m_multiCamReal;
    private final CamOdometryInterface m_multiCamNoop = new NoOpCamOdometry();

    private boolean m_enabled;

    /**
     * Constructor.
     *
     * @param real            The real {@link CamOdometryInterface} to delegate to when enabled.
     * @param initiallyEnabled Whether vision processing starts enabled.
     */
    public MultiCamOdometryWrapper(CamOdometryInterface real, boolean initiallyEnabled) {
        m_multiCamReal = real;
        m_enabled = initiallyEnabled;
    }

    private CamOdometryInterface active() {
        return m_enabled ? m_multiCamReal : m_multiCamNoop;
    }

    @Override
    public Optional<Pose2d> getEstimatedPose() {
        return active().getEstimatedPose();
    }

    @Override
    public Optional<Transform2d> getVisionErrorAtSnapTime() {
        return active().getVisionErrorAtSnapTime();
    }

    @Override
    public double getConfidenceScore() {
        return active().getConfidenceScore();
    }

    @Override
    public List<Integer> getVisibleTagIds() {
        return active().getVisibleTagIds();
    }

    @Override
    public boolean hasTargetLock() {
        return active().hasTargetLock();
    }

    @Override
    public boolean hasMultiTagLock() {
        return active().hasMultiTagLock();
    }

    @Override
    public boolean isLatestMt2() {
        return active().isLatestMt2();
    }

    @Override
    public int getPrimaryTagId() {
        return active().getPrimaryTagId();
    }

    @Override
    public double getPrimaryTagTx() {
        return active().getPrimaryTagTx();
    }

    @Override
    public void setRobotOrientation() {
        active().setRobotOrientation();
    }

    @Override
    public void setRobotOrientation_NoFlush() {
        active().setRobotOrientation_NoFlush();
    }

    @Override
    public void periodic() {
        active().periodic();
    }

    /**
     * Enables or disables vision processing. When disabled, all delegations go to the no-op
     * implementation; the real camera is not updated or queried.
     */
    @Override
    public void enableVision(boolean enabled) {
        m_enabled = enabled;
    }

    /**
     * Always forwarded to the real camera — dependencies must be wired regardless of enable state.
     */
    @Override
    public void setVisionDependenciesOnCamera(
            VisionKalmanFilter filter,
            BooleanSupplier isMotionlessSupplier) {
        m_multiCamReal.setVisionDependenciesOnCamera(filter, isMotionlessSupplier);
    }
}
