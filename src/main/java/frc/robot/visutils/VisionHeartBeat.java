package frc.robot.visutils;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import frc.robot.LimelightHelpers;

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
    private boolean m_hasTid = false;
    private boolean m_hasMt1Pose = false;
    private boolean m_hasMt2Pose = false;

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
     */
    public void update() {
        m_cycleCount++;
        if (m_cycleCount % m_checkInterval != 0) {
            return;
        }

        // --- heartbeat ---
        double newHeartbeat = m_heartbeatSupplier.getAsDouble();
        m_isHeartbeating = !Double.isNaN(m_lastHeartbeat) && (newHeartbeat != m_lastHeartbeat);
        m_lastHeartbeat = newHeartbeat;

        // --- tid ---
        m_hasTid = (m_tidSupplier.getAsDouble() != -1);

        // --- MegaTag1 pose ---
        m_hasMt1Pose = LimelightHelpers.validPoseEstimate(m_mt1Supplier.get());

        // --- MegaTag2 pose ---
        m_hasMt2Pose = LimelightHelpers.validPoseEstimate(m_mt2Supplier.get());
    }

    /** @return True if the Limelight heartbeat counter is incrementing. */
    public boolean isHeartbeating() {
        return m_isHeartbeating;
    }

    /** @return True if a fiducial target has been detected (tid != -1). */
    public boolean hasTid() {
        return m_hasTid;
    }

    /** @return True if MegaTag1 has returned a valid pose estimate. */
    public boolean hasMt1Pose() {
        return m_hasMt1Pose;
    }

    /** @return True if MegaTag2 has returned a valid pose estimate. */
    public boolean hasMt2Pose() {
        return m_hasMt2Pose;
    }
}
