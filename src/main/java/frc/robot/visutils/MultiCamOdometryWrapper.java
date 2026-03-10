package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;


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

    private final BasicInfoDashboard m_dashboard;
    private final MotionlessTracker m_motionlessTracker;

    private boolean m_enabled;

    /**
     * Constructor for production use — wires the dashboard and motionless tracker.
     *
     * @param real              The real {@link CamOdometryInterface} to delegate to when enabled.
     * @param initiallyEnabled  Whether vision processing starts enabled.
     * @param dashboard         Dashboard to push vision state to each cycle.
     * @param motionlessTracker Tracks whether the robot is stationary.
     */
    public MultiCamOdometryWrapper(
            CamOdometryInterface real,
            boolean initiallyEnabled,
            BasicInfoDashboard dashboard,
            MotionlessTracker motionlessTracker) {

        if ((dashboard == null && motionlessTracker != null) ||
            (dashboard != null && motionlessTracker == null)) {
            throw new IllegalArgumentException(
                "dashboard and motionlessTracker must both be null or both be non-null");
        }

        m_multiCamReal = real;
        m_enabled = initiallyEnabled;
        m_dashboard = dashboard;
        m_motionlessTracker = motionlessTracker;
    }

    /**
     * Constructor without dashboard wiring (used in unit tests).
     *
     * @param real             The real {@link CamOdometryInterface} to delegate to when enabled.
     * @param initiallyEnabled Whether vision processing starts enabled.
     */
    public MultiCamOdometryWrapper(CamOdometryInterface real, boolean initiallyEnabled) {
        this(real, initiallyEnabled, null, null);
    }

    private CamOdometryInterface active() {
        return m_enabled ? m_multiCamReal : m_multiCamNoop;
    }

    @Override
    public Optional<Pose2d> getEstimatedPose() {
        return active().getEstimatedPose();
    }

    @Override
    public OptionalDouble getVisionErrorAtSnapTime() {
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
    public void setRobotOrientationNoFlush() {
        active().setRobotOrientationNoFlush();
    }

    @Override
    public void periodic() {
        active().periodic();
        if (m_dashboard != null && m_motionlessTracker != null) {
            m_dashboard.updateVisionConfidence(getConfidenceScore());
            m_dashboard.updateTargetLock(hasTargetLock());
            m_dashboard.updateMultiTagLock(hasMultiTagLock());
            m_dashboard.updateIsLatestMt2(isLatestMt2());
            m_dashboard.updatePrimaryTagTx(getPrimaryTagTx());
            m_dashboard.updateVisibleTagIds(getVisibleTagIds());
            m_dashboard.updateVisionErrorAtSnapTime(getVisionErrorAtSnapTime());
            m_dashboard.updateMotionlessState(
                m_motionlessTracker.isMotionless(), m_motionlessTracker.getSecondsStill());
        }
    }

    /**
     * Enables or disables vision processing. When disabled, all delegations go to the no-op
     * implementation; the real camera is not updated or queried.
     */
    @Override
    public void enableVision(boolean enabled) {
        m_enabled = enabled;
    }
}
