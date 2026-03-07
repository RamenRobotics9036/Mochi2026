package frc.robot.visutils;

import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.kMultiTagStdDevs;
import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.kSingleTagStdDevs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import com.ctre.phoenix6.Utils;
import frc.robot.LimelightHelpers;
import frc.robot.sim.visionproducers.VisionSimInterface;
import java.util.Arrays;
import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;


/**
 * Limelight-based odometry measurement source.
 */
public class SingleCamOdometry implements CamOdometryInterface {
    private final String m_limelightName;
    private VisionSimInterface.EstimateConsumer m_estConsumer;
    private Matrix<N3, N1> m_curStdDevs = kSingleTagStdDevs;
    private double m_lastTimestamp = 0;

    private Optional<Pose2d> m_latestVisPose = Optional.empty();
    private Optional<Transform2d> m_latestVisionError = Optional.empty();
    private double m_curConfidenceScore = 0.0;
    // Booleans mirror the CamOdometryInterface contract directly;
    // no consumer needs the raw tag count.
    private boolean m_hasTargetLock = false;
    private boolean m_hasMultiTagLock = false;
    private double m_tx = 0.0;
    private List<Integer> m_targetList = Collections.emptyList();
    private int m_lastTarget = -1;

    private VisionKalmanFilter m_visionKalmanFilter = null;
    private BooleanSupplier m_isMotionlessSupplier = null;
    private BooleanSupplier m_visionEnabledSupplier = () -> true;

    // Samples the drivetrain's historical pose at a given FPGA timestamp (seconds)
    private final Function<Double, Optional<Pose2d>> m_poseSampler;

    // Supplies the robot's current heading (degrees, WPILib blue-alliance frame) for MegaTag2
    private final DoubleSupplier m_yawDegreesSupplier;

    /** Constructor. */
    public SingleCamOdometry(
        String limelightName,
        Transform3d robotToCam,
        VisionSimInterface.EstimateConsumer poseConsumer,
        Function<Double, Optional<Pose2d>> poseSampler,
        DoubleSupplier yawDegreesSupplier) {

        m_estConsumer = poseConsumer;
        m_limelightName = limelightName;
        m_poseSampler = poseSampler;
        m_yawDegreesSupplier = yawDegreesSupplier;

        setCameraPoseRobotSpace(m_limelightName, robotToCam);
    }

    private void setCameraPoseRobotSpace(String limelightName, Transform3d robotToCam) {
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            robotToCam.getX(),
            robotToCam.getY(),
            robotToCam.getZ(),
            Math.toDegrees(robotToCam.getRotation().getX()),
            Math.toDegrees(robotToCam.getRotation().getY()),
            Math.toDegrees(robotToCam.getRotation().getZ())
        );
    }

    /**
     * Calculates the offset between where the drivetrain thought the robot was
     * at the camera snap timestamp and where vision says it is now.
     *
     * <p>Calls {@code m_poseSampler} (i.e. {@code drivetrain::samplePoseAt}) with
     * the FPGA timestamp converted to the {@link Utils#getCurrentTimeSeconds()} epoch,
     * then returns the {@link Transform2d} from that sampled pose to {@code visionPose}
     * (translation + rotation delta).
     *
     * @param visionPose           The pose reported by vision for this measurement
     * @param fpgaTimestampSeconds FPGA timestamp (seconds) when the image was captured
     *                             (i.e. {@code mt1.timestampSeconds})
     * @return The Transform2d offset, or empty if no sampler is set or the
     *         pose buffer has no entry for that timestamp
     */
    private Optional<Transform2d> calcVisionErrorAtSnapTime(
            Pose2d visionPose, double fpgaTimestampSeconds) {
        if (m_poseSampler == null) {
            return Optional.empty();
        }

        // samplePoseAt requires getCurrentTimeSeconds epoch, not FPGA epoch
        double currentTimeEpoch = Utils.fpgaToCurrentTime(fpgaTimestampSeconds);
        Optional<Pose2d> sampledPose = m_poseSampler.apply(currentTimeEpoch);
        if (sampledPose.isEmpty()) {
            return Optional.empty();
        }

        // Transform2d from sampledPose → visionPose (translation + rotation offset)
        Optional<Transform2d> result = Optional.of(visionPose.minus(sampledPose.get()));

        // System.out.println(String.format("Vision offset at snap time: (x=%.2f m, y=%.2f m, rot=%.1f°)",
        //     result.get().getX(), result.get().getY(), result.get().getRotation().getDegrees()));

        return result;
    }

    /**
     * Sets the dependencies needed for vision processing.
     *
     * @param visionEnabledSupplier A BooleanSupplier returning true when vision is enabled
     * @param filter The VisionKalmanFilter instance to inject measurements into
     * @param isMotionlessSupplier Supplier that returns true when robot is motionless
     */
    @Override
    public void setVisionDependencies(
            BooleanSupplier visionEnabledSupplier,
            VisionKalmanFilter filter,
            BooleanSupplier isMotionlessSupplier) {
        m_visionEnabledSupplier = visionEnabledSupplier;
        m_visionKalmanFilter = filter;
        m_isMotionlessSupplier = isMotionlessSupplier;
    }

    @Override
    public void periodic() {
        addVisionMeasurementV1();
    }

    private void clearResults() {
        m_curConfidenceScore = 0.0;
        m_hasTargetLock = false;
        m_hasMultiTagLock = false;
        m_tx = 0.0;
        m_targetList = Collections.emptyList();
        m_latestVisionError = Optional.empty();
    }

    private void setResults(double confidenceScore, int numLockedTags,
                            LimelightHelpers.RawFiducial[] rawFiducials) {
        m_curConfidenceScore = confidenceScore;
        m_hasTargetLock = numLockedTags > 0;
        m_hasMultiTagLock = numLockedTags > 1;

        // Horizontal offset to primary target (degrees)
        m_tx = LimelightHelpers.getTX(m_limelightName);

        // Build list of visible tag IDs
        if (rawFiducials != null && rawFiducials.length > 0) {
            m_targetList = Arrays.stream(rawFiducials)
                .map(f -> f.id)
                .collect(Collectors.toList());
            m_lastTarget = rawFiducials[0].id;
        }
        else {
            m_targetList = Collections.emptyList();
        }
    }

    // Optional method to help debug limelight vision
    @SuppressWarnings("unused")
    private void printDebugLimelightInfo(LimelightHelpers.PoseEstimate mt1) {
        StringBuilder sb = new StringBuilder("Camera info: ");

        // Horizontal offset to primary target (degrees)
        double tempTx = LimelightHelpers.getTX(m_limelightName);
        sb.append(String.format("tx=%7.2f°", tempTx));

        double tempId = LimelightHelpers.getFiducialID(m_limelightName);
        sb.append(String.format(", ID=%4s", tempId >= 0 ? String.valueOf((int) tempId) : "None"));

        if (mt1 == null) {
            sb.append(", No pose estimate");
        }
        else {
            sb.append(String.format(", tags=%d, pose=(%6.2f, %6.2f, %7.1f°)",
                mt1.tagCount,
                mt1.pose.getX(),
                mt1.pose.getY(),
                mt1.pose.getRotation().getDegrees()));
        }

        // Try getting RAW fiducials directly from NetworkTables
        LimelightHelpers.RawFiducial[] fiducials =
            LimelightHelpers.getRawFiducials(m_limelightName);
        if (fiducials.length > 0) {
            String ids = Arrays.stream(fiducials)
                .map(f -> String.valueOf(f.id))
                .collect(Collectors.joining(", "));
            sb.append(", rawFiducials=[" + ids + "]");
        }

        System.out.println(sb.toString());
    }

    private void addVisionMeasurementV1() {
        // MegaTag2 requires the robot's current heading to be pushed to the Limelight
        // before each poll so it can fuse gyro data with its visual solve.
        LimelightHelpers.SetRobotOrientation_NoFlush(
            m_limelightName, m_yawDegreesSupplier.getAsDouble(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt1 = sanitizePoseEstimate(
            LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName));
        LimelightHelpers.PoseEstimate mt2 = sanitizePoseEstimate(
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName));

        // Prefer MegaTag2 when available, fall back to MegaTag1.
        LimelightHelpers.PoseEstimate poseEstimate = mt2 != null ? mt2 : mt1;

        // printDebugLimelightInfo(poseEstimate);

        // Save the latest vision estimate so that it can be queried
        m_latestVisPose = Optional.ofNullable(poseEstimate).map(est -> est.pose);

        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            // In simulation, limelight may not be present until a few cycles of periodic, since we
            // populate it via NetworkTables later.
            clearResults();
            return;
        }

        // Skip if this is the same data we already processed
        if (poseEstimate.timestampSeconds == m_lastTimestamp) {
            return;
        }
        m_lastTimestamp = poseEstimate.timestampSeconds;

        // We track how far-off this vision estimate is, using the ACTUAL pose in the past
        // of where the robot was when the camera image was snapped.
        m_latestVisionError =
            calcVisionErrorAtSnapTime(poseEstimate.pose, poseEstimate.timestampSeconds);

        // Update std devs based on tag count and distance.  And confidence score.
        m_curStdDevs = calculateEstimationStdDevs(poseEstimate);
        m_curConfidenceScore = getConfidenceScore(m_curStdDevs);

        // Ambiguity only matters for MegaTag1 (pure visual PnP); MT2 resolves ambiguity
        // using the gyro heading, so this check must not apply to MT2 estimates.
        if (!poseEstimate.isMegaTag2 && poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
            if (poseEstimate.rawFiducials[0].ambiguity > 0.7) {
                setResults(m_curConfidenceScore, 0, null);
                return;
            }
        }

        // Check if std devs indicate rejection
        if (m_curStdDevs.get(0, 0) == Double.MAX_VALUE) {
            setResults(m_curConfidenceScore, 0, null);
            return;
        }

        setResults(m_curConfidenceScore, poseEstimate.tagCount, poseEstimate.rawFiducials);

        // Skip injecting vision measurements if vision is disabled
        if (!m_visionEnabledSupplier.getAsBoolean()) {
            return;
        }

        // Inject into vision Kalman filter if robot is motionless and we have multi-tag
        if (m_visionKalmanFilter != null && m_isMotionlessSupplier != null) {
            if (m_isMotionlessSupplier.getAsBoolean() && poseEstimate.tagCount >= 2) {
                m_visionKalmanFilter.injectVisionMeasurement(
                    poseEstimate.pose, poseEstimate.tagCount);
            }
        }

        if (m_estConsumer != null) {
            m_estConsumer.accept(
                poseEstimate.pose, poseEstimate.timestampSeconds, m_curStdDevs);
        }
    }

    private LimelightHelpers.PoseEstimate sanitizePoseEstimate(
            LimelightHelpers.PoseEstimate poseEstimate) {
        // LimelightHelpers 2026.1 no longer returns null when no targets are visible.
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            return null;
        }
        return poseEstimate;
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic std
     * deviations based on number of tags and distance from the tags.
     *
     * @param poseEstimate The Limelight pose estimate to evaluate
     * @return The calculated standard deviations matrix
     */
    private Matrix<N3, N1> calculateEstimationStdDevs(LimelightHelpers.PoseEstimate poseEstimate) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            // No pose input. Default to single-tag std devs
            return kSingleTagStdDevs;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagStdDevs;
        int numTags = poseEstimate.tagCount;
        double avgDist = poseEstimate.avgTagDist;

        // One or more tags visible, run the full heuristic.
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = kMultiTagStdDevs;
        }

        // Increase std devs based on (average) distance.
        // MT1 single-tag poses become unreliable past 4 m due to PnP ambiguity; reject them.
        // MT2 is gyro-fused and does not have this problem, so the cutoff is skipped for it.
        if (!poseEstimate.isMegaTag2 && numTags == 1 && avgDist > 4) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        Matrix<N3, N1> result = estStdDevs.times(1 + (avgDist * avgDist / 30));
        if (poseEstimate.isMegaTag2) {
            // MT2's reported rotation is the gyro heading reflected back — it carries no new
            // rotational information, so we set theta std dev to a very large value so the
            // pose estimator ignores the rotational component of this measurement.
            return VecBuilder.fill(result.get(0, 0), result.get(1, 0), 9999999.0);
        }
        return result;
    }

    /**
     * Converts std devs to a 0-100 confidence score.
     *
     * @param stdDevs The (x, y, theta) standard deviations matrix
     * @return Confidence from 0 (no confidence) to 100 (highest)
     */
    private double getConfidenceScore(Matrix<N3, N1> stdDevs) {
        // Handle rejection case
        if (stdDevs.get(0, 0) >= Double.MAX_VALUE) {
            return 0.0;
        }

        // Combine position uncertainties (Euclidean norm of x,y)
        double posUncertainty = Math.hypot(stdDevs.get(0, 0), stdDevs.get(1, 0));

        // Map to 0-100 using exponential decay
        // At ~0.7m combined uncertainty → ~100% confidence
        // At ~5m combined uncertainty → ~0% confidence
        double confidence = 100.0 * Math.exp(-posUncertainty / 2.0);

        return Math.max(0, Math.min(100, confidence));
    }

    @Override
    public Optional<Transform2d> getVisionErrorAtSnapTime() {
        return m_latestVisionError;
    }

    @Override
    public Optional<Pose2d> getEstimatedPose() {
        return m_latestVisPose;
    }

    @Override
    public double getConfidenceScore() {
        return m_curConfidenceScore;
    }

    @Override
    public boolean hasTargetLock() {
        return m_hasTargetLock;
    }

    @Override
    public boolean hasMultiTagLock() {
        return m_hasMultiTagLock;
    }

    @Override
    public double getPrimaryTagTx() {
        return m_tx;
    }

    @Override
    public List<Integer> getVisibleTagIds() {
        return m_targetList;
    }

    /** Returns the ID of the last seen fiducial, or -1 if none. */
    @Override
    public int getPrimaryTagId() {
        return m_lastTarget;
    }
}
