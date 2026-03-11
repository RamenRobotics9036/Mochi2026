package frc.robot.visutils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.sim.visionproducers.VisionSimInterface;
import frc.robot.util.MathUtils;
import frc.robot.visutils.evaluateposes.EvaluatePosesInterface;
import java.util.Arrays;
import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;


/**
 * Limelight-based odometry measurement source.
 */
public class SingleCamOdometry implements CamOdometryInterface {
    private final boolean m_megaTag2Enabled;
    private final boolean m_autoVisionInjectionEnabled;

    private final String m_limelightName;
    private VisionSimInterface.EstimateConsumer m_estConsumer;
    private double m_lastTimestamp = 0;

    private Optional<Pose2d> m_latestVisPose = Optional.empty();
    private Optional<Transform2d> m_latestVisionError = Optional.empty();
    private double m_curConfidenceScore = 0.0;
    // Booleans mirror the CamOdometryInterface contract directly;
    // no consumer needs the raw tag count.
    private boolean m_hasTargetLock = false;
    private boolean m_hasMultiTagLock = false;
    private boolean m_isLatestMt2 = false;
    private double m_tx = 0.0;
    private List<Integer> m_targetList = Collections.emptyList();
    private int m_lastTarget = -1;

    private VisionKalmanFilter m_visionKalmanFilter = null;
    private BooleanSupplier m_isMotionlessSupplier = null;

    private final EvaluatePosesInterface m_evaluatePoses;

    // Lambda to query current robot pose
    private final Supplier<Pose2d> m_currentRobotPoseSupplier;

    // Samples the drivetrain's historical pose at a given FPGA timestamp (seconds)
    private final Function<Double, Optional<Pose2d>> m_poseSampler;

    // Supplies the robot's current heading (degrees, WPILib blue-alliance frame) for MegaTag2
    private final DoubleSupplier m_yawDegreesSupplier;

    /** Constructor. */
    public SingleCamOdometry(
        String limelightName,
        Transform3d robotToCam,
        VisionSimInterface.EstimateConsumer poseConsumer,
        Supplier<Pose2d> currentRobotPoseSupplier,
        Function<Double, Optional<Pose2d>> poseSampler,
        DoubleSupplier yawDegreesSupplier,
        boolean megaTag2Enabled,
        boolean autoVisionInjectionEnabled,
        EvaluatePosesInterface evaluatePoses) {

        m_estConsumer = poseConsumer;
        m_limelightName = limelightName;
        m_poseSampler = poseSampler;
        m_currentRobotPoseSupplier = currentRobotPoseSupplier;
        m_yawDegreesSupplier = yawDegreesSupplier;
        m_megaTag2Enabled = megaTag2Enabled;
        m_autoVisionInjectionEnabled = autoVisionInjectionEnabled;
        m_evaluatePoses = evaluatePoses;

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
    // $TODO2 - Get vision error at SnapTime
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
     * @param filter The VisionKalmanFilter instance to inject measurements into
     * @param isMotionlessSupplier Supplier that returns true when robot is motionless
     */
    @Override
    public void setVisionDependenciesOnCamera(
            VisionKalmanFilter filter,
            BooleanSupplier isMotionlessSupplier) {
        m_visionKalmanFilter = filter;
        m_isMotionlessSupplier = isMotionlessSupplier;
    }

    @Override
    public void periodic() {
        addVisionMeasurementV1();
    }

    @Override
    public void enableVision(boolean enabled) {
        throw new UnsupportedOperationException("Vision enabling/disabling is not supported at the SingleCamOdometry level; enable/disable the entire wrapper instead.");
    }

    private void clearResults() {
        m_curConfidenceScore = 0.0;
        m_hasTargetLock = false;
        m_hasMultiTagLock = false;
        m_isLatestMt2 = false;
        m_tx = 0.0;
        m_targetList = Collections.emptyList();
        m_latestVisionError = Optional.empty();
    }

    private void setResults(double confidenceScore, int numLockedTags,
                            LimelightHelpers.RawFiducial[] rawFiducials, boolean isMegaTag2) {
        m_curConfidenceScore = confidenceScore;
        m_hasTargetLock = numLockedTags > 0;
        m_hasMultiTagLock = numLockedTags > 1;
        m_isLatestMt2 = isMegaTag2 && numLockedTags > 0;

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

    @Override
    public void setRobotOrientation() {
        if (isMegaTag2Enabled()) {
            LimelightHelpers.SetRobotOrientation(
                m_limelightName,
                m_yawDegreesSupplier.getAsDouble(),
                0,
                0,
                0,
                0,
                0);
        }
    }

    @Override
    public void setRobotOrientation_NoFlush() {
        if (isMegaTag2Enabled()) {
            LimelightHelpers.SetRobotOrientation_NoFlush(
                m_limelightName,
                m_yawDegreesSupplier.getAsDouble(),
                0,
                0,
                0,
                0,
                0);
        }
    }

    // $TODO2 - Code that checks whether to discard bad estimates
    private void addVisionMeasurementV1() {
        PoseEstimate mt1 = sanitizePoseEstimate(
            LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName));
        PoseEstimate mt2 = isMegaTag2Enabled()
            ? sanitizePoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName))
            : null;

        PoseEstimate poseEstimate = m_evaluatePoses.pickMegatag1vsMegatag2(mt1, mt2);

        // Save the latest vision estimate so that it can be queried
        m_latestVisPose = Optional.ofNullable(poseEstimate).map(est -> est.pose);

        if (m_evaluatePoses.isVisionPoseBad(poseEstimate, m_currentRobotPoseSupplier.get())) {
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
        Matrix<N3, N1> curStdDevs = m_evaluatePoses.calcVisionPoseStdDev(poseEstimate);
        m_curConfidenceScore = m_evaluatePoses.calcVisionPoseScore(curStdDevs);

        // Ambiguity only matters for MegaTag1 (pure visual PnP); MT2 resolves ambiguity
        // using the gyro heading, so this check must not apply to MT2 estimates.
        if (!poseEstimate.isMegaTag2 && poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
            if (poseEstimate.rawFiducials[0].ambiguity > 0.7) {
                setResults(m_curConfidenceScore, 0, null, false);
                return;
            }
        }

        // Check if std devs indicate rejection
        if (curStdDevs.get(0, 0) == Double.MAX_VALUE) {
            setResults(m_curConfidenceScore, 0, null, false);
            return;
        }

        setResults(m_curConfidenceScore, poseEstimate.tagCount, poseEstimate.rawFiducials, poseEstimate.isMegaTag2);

        // Inject into vision Kalman filter if robot is motionless and we have multi-tag
        if (m_visionKalmanFilter != null && m_isMotionlessSupplier != null) {
            if (m_isMotionlessSupplier.getAsBoolean() && poseEstimate.tagCount >= 2) {
                m_visionKalmanFilter.injectVisionMeasurement(
                    poseEstimate.pose, poseEstimate.tagCount);
            }
        }

        // $We should ONLY inject vision measurements in AUTO.
        if (m_autoVisionInjectionEnabled && !DriverStation.isTeleopEnabled()) {

            if (m_estConsumer != null) {
                m_estConsumer.accept(
                    poseEstimate.pose, poseEstimate.timestampSeconds, curStdDevs);
            }
        }
    }

    private PoseEstimate sanitizePoseEstimate(
            PoseEstimate poseEstimate) {
        // LimelightHelpers 2026.1 no longer returns null when no targets are visible.
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            return null;
        }
        return poseEstimate;
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
    public boolean isLatestMt2() {
        return m_isLatestMt2;
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

    private boolean isMegaTag2Enabled() {
        return m_megaTag2Enabled;
    }
}
