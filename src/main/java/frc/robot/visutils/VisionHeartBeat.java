package frc.robot.visutils;

import frc.robot.LimelightHelpers;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


/**
 * Lightweight diagnostic that polls a single Limelight camera every N periodic cycles
 * and exposes four boolean status flags:
 * <ol>
 *   <li>Whether the Limelight heartbeat counter is incrementing.</li>
 *   <li>Whether a fiducial target has been detected (tid != -1).</li>
 *   <li>Whether MegaTag1 has returned a valid pose estimate.</li>
 *   <li>Whether MegaTag2 has returned a valid pose estimate.</li>
 * </ol>
 *
 * <p>Call {@link #update()} once per robot periodic cycle.
 */
public class VisionHeartBeat {

    private static final int DEFAULT_CHECK_INTERVAL = 20;

    private final int m_checkInterval;
    private final DoubleSupplier m_heartbeatSupplier;
    private final DoubleSupplier m_tidSupplier;
    private final Supplier<LimelightHelpers.PoseEstimate> m_mt1Supplier;
    private final Supplier<LimelightHelpers.PoseEstimate> m_mt2Supplier;

    // Initialize to (checkInterval - 1) so the very first update() triggers a real check
    // rather than waiting a full interval before providing any status.
    private int m_cycleCount;
    private double m_lastHeartbeat = Double.NaN;

    // Cached results — all start false until the first real check fires
    private boolean m_isHeartbeating = false;
    // Latching flags — flip to true once seen; reset to false when heartbeat is lost
    private boolean m_hasSeenTid = false;
    private boolean m_hasSeenMt1Pose = false;
    private boolean m_hasSeenMt2Pose = false;

    /**
     * Production constructor. Reads from NetworkTables via {@link LimelightHelpers}.
     * Checks every {@value #DEFAULT_CHECK_INTERVAL} cycles (~400 ms at 50 Hz).
     *
     * @param limelightName Name/identifier of the Limelight camera.
     */
    public VisionHeartBeat(String limelightName) {
        this(limelightName, DEFAULT_CHECK_INTERVAL);
    }

    /**
     * Production constructor with configurable check interval.
     *
     * @param limelightName       Name/identifier of the Limelight camera.
     * @param checkIntervalCycles How many {@link #update()} calls to skip between real checks.
     */
    public VisionHeartBeat(String limelightName, int checkIntervalCycles) {
        this(
            () -> LimelightHelpers.getHeartbeat(limelightName),
            () -> LimelightHelpers.getFiducialID(limelightName),
            () -> LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName),
            () -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName),
            checkIntervalCycles);
    }

    /**
     * Testable constructor. Accepts suppliers so tests can inject fake values
     * without requiring NetworkTables.
     */
    VisionHeartBeat(
            DoubleSupplier heartbeatSupplier,
            DoubleSupplier tidSupplier,
            Supplier<LimelightHelpers.PoseEstimate> mt1Supplier,
            Supplier<LimelightHelpers.PoseEstimate> mt2Supplier,
            int checkIntervalCycles) {
        m_heartbeatSupplier = heartbeatSupplier;
        m_tidSupplier = tidSupplier;
        m_mt1Supplier = mt1Supplier;
        m_mt2Supplier = mt2Supplier;
        m_checkInterval = checkIntervalCycles;
        m_cycleCount = checkIntervalCycles - 1;
    }

    /**
     * Call once per robot periodic cycle. Only performs real reads every
     * {@code checkIntervalCycles} calls; otherwise returns cached values.
     *
     * @return {@code true} if a real check was performed this cycle (state may have changed),
     *         {@code false} if this was a skipped cycle (cached state is unchanged).
     */
    public boolean update() {
        m_cycleCount++;
        if (m_cycleCount % m_checkInterval != 0) {
            return false;
        }

        // --- heartbeat ---
        double newHeartbeat = m_heartbeatSupplier.getAsDouble();
        m_isHeartbeating = !Double.isNaN(m_lastHeartbeat) && (newHeartbeat != m_lastHeartbeat);
        m_lastHeartbeat = newHeartbeat;

        // If heartbeat is lost, reset all latches on this same check
        if (!m_isHeartbeating) {
            m_hasSeenTid = false;
            m_hasSeenMt1Pose = false;
            m_hasSeenMt2Pose = false;
            return true;
        }

        // --- tid (latch: stop checking once true) ---
        if (!m_hasSeenTid) {
            m_hasSeenTid = (m_tidSupplier.getAsDouble() != -1);
        }

        // --- MegaTag1 pose (latch: stop checking once true) ---
        if (!m_hasSeenMt1Pose) {
            m_hasSeenMt1Pose = LimelightHelpers.validPoseEstimate(m_mt1Supplier.get());
        }

        // --- MegaTag2 pose (latch: stop checking once true) ---
        if (!m_hasSeenMt2Pose) {
            m_hasSeenMt2Pose = LimelightHelpers.validPoseEstimate(m_mt2Supplier.get());
        }

        return true;
    }

    /** True if the Limelight heartbeat counter is incrementing. */
    public boolean isHeartbeating() {
        return m_isHeartbeating;
    }

    /**
     * True if a fiducial target has ever been seen (tid != -1) since last heartbeat.
     * Latches true.
     */
    public boolean hasSeenTid() {
        return m_hasSeenTid;
    }

    /** True if MegaTag1 has ever returned a valid pose since last heartbeat. Latches true. */
    public boolean hasSeenMt1Pose() {
        return m_hasSeenMt1Pose;
    }

    /** True if MegaTag2 has ever returned a valid pose since last heartbeat. Latches true. */
    public boolean hasSeenMt2Pose() {
        return m_hasSeenMt2Pose;
    }

    /**
     * Returns a human-readable camera status summary.
     *
     * <ul>
     *   <li>{@code "No heartbeat"} — Limelight is not responding</li>
     *   <li>{@code "Seen Tid, Mt1"} — heartbeating, seen TID and MT1 but not MT2</li>
     *   <li>{@code "Seen Tid, Mt2"} — heartbeating, seen TID and MT2 but not MT1</li>
     *   <li>{@code "Healthy"} — heartbeating, seen TID, MT1, and MT2</li>
     * </ul>
     */
    public String getCameraStatus() {
        if (!m_isHeartbeating) {
            return "No heartbeat";
        }

        if (m_hasSeenTid && m_hasSeenMt1Pose && m_hasSeenMt2Pose) {
            return "Healthy!";
        }
        if (m_hasSeenTid && m_hasSeenMt1Pose) {
            return "Seen Id, Mt1";
        }
        if (m_hasSeenTid && m_hasSeenMt2Pose) {
            return "Seen Id, Mt2";
        }
        if (m_hasSeenTid) {
            return "Seen Id";
        }

        return "Heartbeat only...";
    }
}
