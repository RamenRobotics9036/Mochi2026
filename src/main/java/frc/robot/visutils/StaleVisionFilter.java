package frc.robot.visutils;

import com.ctre.phoenix6.Utils;

/**
 * Rejects vision measurements whose timestamps fall within a window around the
 * most recent pose reset, preventing stale camera frames from corrupting the
 * freshly reset pose.
 *
 * <p><b>Why stale frames are a problem:</b><br>
 * Camera frames are captured asynchronously. A frame snapped at T=1.0s may still
 * be in the pipeline when the pose is reset at T=1.05s. If that frame is processed
 * after the reset, the pose estimator's backward latency-compensation will pull the
 * new pose back toward where the robot was before the reset.
 *
 * <p><b>The fix:</b><br>
 * Any measurement whose (converted) timestamp falls within
 * {@code [resetTime - 2 s, resetTime + 0 s]} is discarded. This covers frames that
 * were already in-flight when the reset occurred and gives the pose estimator time
 * to stabilize before vision corrections resume.
 */
public class StaleVisionFilter {
    /** Sentinel value indicating no pose reset has occurred yet, allowing vision to work immediately on startup */
    private static final double RESET_INIT_CONSTANT = -1.0;

    /** How far back in time (seconds) before reset to start ignoring vision measurements */
    private static final double IGNORE_WINDOW_START_OFFSET = 2.0;

    /** How far forward in time (seconds) after reset to stop ignoring vision measurements */
    private static final double IGNORE_WINDOW_END_OFFSET = 0.0;

    /** Track the last time the pose was reset to filter stale vision measurements. */
    private double m_lastResetTimestamp = RESET_INIT_CONSTANT;

    /**
     * Records that a pose reset has occurred at the current time.
     * Call this when the drivetrain pose is reset.
     *
     * @param currentTimeSeconds The current time in seconds (from Utils.getCurrentTimeSeconds())
     */
    public void recordPoseReset(double currentTimeSeconds) {
        m_lastResetTimestamp = currentTimeSeconds;
    }

    private boolean shouldIgnoreOldTimestamps(double timestampSeconds) {
        if (m_lastResetTimestamp == RESET_INIT_CONSTANT) {
            return false;
        }

        double convertedTimestamp = Utils.fpgaToCurrentTime(timestampSeconds);
        double ignoreWindowStart = m_lastResetTimestamp - IGNORE_WINDOW_START_OFFSET;
        double ignoreWindowEnd = m_lastResetTimestamp + IGNORE_WINDOW_END_OFFSET;

        return convertedTimestamp >= ignoreWindowStart && convertedTimestamp <= ignoreWindowEnd;
    }

    /**
     * Determines if a vision measurement should be ignored based on its timestamp
     * relative to the last pose reset.
     *
     * @param timestampSeconds The timestamp of the vision measurement in seconds (FPGA time).
     * @return true if the measurement should be ignored, false otherwise.
     */
    public boolean shouldIgnore(
        double timestampSeconds) {

        return shouldIgnoreOldTimestamps(timestampSeconds);
    }
}
