package frc.robot.visutils;

import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.kMultiTagStdDevs;
import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.kSingleTagStdDevs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.sim.visionproducers.VisionSimInterface;
import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Collectors;


/**
 * Limelight-based odometry measurement source.
 */
public class LimelightOdometry {
    private final String m_limelightName;
    private VisionSimInterface.EstimateConsumer m_estConsumer;
    private Matrix<N3, N1> m_curStdDevs = kSingleTagStdDevs;
    private double m_lastTimestamp = 0;

    private Optional<Pose2d> m_latestVisPose = Optional.empty();
    private double m_curConfidenceScore = 0.0;
    private int m_numLockedTags = 0;
    private double m_tx = 0.0;
    private String m_targetList = "";

    private VisionKalmanFilter m_visionKalmanFilter = null;
    private BooleanSupplier m_isMotionlessSupplier = null;

    /** Constructor. */
    public LimelightOdometry(VisionSimInterface.EstimateConsumer poseConsumer) {
        this.m_estConsumer = poseConsumer;
        this.m_limelightName = Robot.isSimulation()
            ? VisionConstants.kLimelightNameSim
            : VisionConstants.kLimelightNameReal;
    }

    /**
     * Sets the vision Kalman filter and motionless detection for precise stationary estimation.
     *
     * @param filter The VisionKalmanFilter instance to inject measurements into
     * @param isMotionlessSupplier Supplier that returns true when robot is motionless
     */
    public void setVisionKalmanFilter(VisionKalmanFilter filter, BooleanSupplier isMotionlessSupplier) {
        m_visionKalmanFilter = filter;
        m_isMotionlessSupplier = isMotionlessSupplier;
    }

    /** Periodic update; should be called from robot periodic. */
    public void periodic() {
        addVisionMeasurementV1();
    }

    private void clearResults() {
        m_curConfidenceScore = 0.0;
        m_numLockedTags = 0;
        m_tx = 0.0;
        m_targetList = "";
    }

    private void setResults(double confidenceScore, int numLockedTags,
                            LimelightHelpers.RawFiducial[] rawFiducials) {
        m_curConfidenceScore = confidenceScore;
        m_numLockedTags = numLockedTags;

        // Horizontal offset to primary target (degrees)
        m_tx = LimelightHelpers.getTX(m_limelightName);

        // Build comma-separated list of visible tag IDs
        if (rawFiducials != null && rawFiducials.length > 0) {
            m_targetList = Arrays.stream(rawFiducials)
                .map(f -> String.valueOf(f.id))
                .collect(Collectors.joining(", "));
        } else {
            m_targetList = "";
        }
    }

    // Optional method to help debug limelight vision
    @SuppressWarnings("unused")
    private void printDebugLimelightInfo(LimelightHelpers.PoseEstimate mt1) {
        StringBuilder sb = new StringBuilder("LimelightOdometry: ");

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
        LimelightHelpers.PoseEstimate mt1 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);

        // Limelighthelpers 2026.1 no longer returns null when no targets are visible.
        // So need to sanitize mt1 value so mt1 == null is still "no targets".
        if (mt1 == null || mt1.tagCount == 0) {
            mt1 = null;
        }

        // printDebugLimelightInfo(mt1);

        // Save the latest vision estimate so that it can be queried
        m_latestVisPose = Optional.ofNullable(mt1).map(est -> est.pose);

        if (mt1 == null || mt1.tagCount == 0) {
            // In simulation, limelight may not be present until a few cycles of periodic, since we
            // populate it via NetworkTables later.
            clearResults();
            return;
        }

        // Skip if this is the same data we already processed
        if (mt1.timestampSeconds == m_lastTimestamp) {
            return;
        }
        m_lastTimestamp = mt1.timestampSeconds;

        // Update std devs based on tag count and distance.  And confidence score.
        m_curStdDevs = calculateEstimationStdDevs(mt1);
        m_curConfidenceScore = getConfidenceScore(m_curStdDevs);

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > 0.7) {
                setResults(m_curConfidenceScore, 0, null);
                return;
            }
        }

        // Check if std devs indicate rejection
        if (m_curStdDevs.get(0, 0) == Double.MAX_VALUE) {
            setResults(m_curConfidenceScore, 0, null);
            return;
        }

        setResults(m_curConfidenceScore, mt1.tagCount, mt1.rawFiducials);

        // Inject into vision Kalman filter if robot is motionless and we have multi-tag
        if (m_visionKalmanFilter != null && m_isMotionlessSupplier != null) {
            if (m_isMotionlessSupplier.getAsBoolean() && mt1.tagCount >= 2) {
                m_visionKalmanFilter.injectVisionMeasurement(mt1.pose, mt1.tagCount);
            }
        }

        if (m_estConsumer != null) {
            m_estConsumer.accept(mt1.pose, mt1.timestampSeconds, m_curStdDevs);
        }
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

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        return estStdDevs.times(1 + (avgDist * avgDist / 30));
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

    public Optional<Pose2d> getLatestVisPose() {
        return m_latestVisPose;
    }

    public double getCurrentConfidenceScore() {
        return m_curConfidenceScore;
    }

    public int getNumLockedTags() {
        return m_numLockedTags;
    }

    public double getTx() {
        return m_tx;
    }

    public String getTargetList() {
        return m_targetList;
    }
}
