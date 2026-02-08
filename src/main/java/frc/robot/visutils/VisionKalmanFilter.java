package frc.robot.visutils;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionKalmanConstants;

/**
 * A vision-only Kalman filter for accurately averaging robot position
 * when the robot is motionless and multiple AprilTags are visible.
 *
 * <p>This filter ignores odometry/gyro and only uses multi-tag vision
 * measurements, making it ideal for precise position estimation during
 * stationary alignment or autonomous setup.
 *
 * <p>The filter converges over time as more measurements are taken,
 * with the covariance matrix P shrinking to indicate increased certainty.
 */
public class VisionKalmanFilter {

    /**
     * Bundles the Kalman filter's display state for field visualization.
     *
     * @param pose The display pose (with forward offset applied), or empty if not initialized
     * @param hasConverged Whether the filter has converged
     */
    public record DisplayInfo(Optional<Pose2d> pose, boolean hasConverged) {}

    /** State vector: [x, y, θ] */
    private Matrix<N3, N1> m_x;

    /** State covariance matrix */
    private Matrix<N3, N3> m_P;

    /** Whether the filter has received its first measurement */
    private boolean m_initialized = false;

    /** Number of measurements injected since last reset */
    private int m_measurementCount = 0;

    /**
     * Injects a vision measurement into the filter.
     * Measurements with fewer than 2 tags are ignored.
     *
     * @param pose The measured robot pose from vision
     * @param tagCount Number of AprilTags used in this measurement
     */
    public void injectVisionMeasurement(Pose2d pose, int tagCount) {
        // Reject single-tag measurements (require MegaTag accuracy)
        if (tagCount < 2) {
            return;
        }

        if (!m_initialized) {
            // First measurement initializes the state
            m_x = new Matrix<>(Nat.N3(), Nat.N1());
            m_x.set(0, 0, pose.getX());
            m_x.set(1, 0, pose.getY());
            m_x.set(2, 0, pose.getRotation().getRadians());

            m_P = createInitialCovariance();
            m_initialized = true;
            m_measurementCount = 1;
            return;
        }

        // Create measurement vector z
        Matrix<N3, N1> z = new Matrix<>(Nat.N3(), Nat.N1());
        z.set(0, 0, pose.getX());
        z.set(1, 0, pose.getY());
        z.set(2, 0, pose.getRotation().getRadians());

        // Compute measurement noise R (scales inversely with tag count)
        Matrix<N3, N3> R = computeMeasurementNoise(tagCount);

        // Kalman update equations (H = I for direct pose measurement)
        // Innovation: y = z - x
        Matrix<N3, N1> y = z.minus(m_x);

        // Handle angle wrapping for theta component
        double thetaError = y.get(2, 0);
        while (thetaError > Math.PI) thetaError -= 2 * Math.PI;
        while (thetaError < -Math.PI) thetaError += 2 * Math.PI;
        y.set(2, 0, thetaError);

        // Innovation covariance: S = P + R (since H = I)
        Matrix<N3, N3> S = m_P.plus(R);

        // Kalman gain: K = P * S^-1
        Matrix<N3, N3> K = m_P.times(S.inv());

        // State update: x = x + K * y
        m_x = m_x.plus(K.times(y));

        // Normalize theta to [-π, π]
        double theta = m_x.get(2, 0);
        while (theta > Math.PI) theta -= 2 * Math.PI;
        while (theta < -Math.PI) theta += 2 * Math.PI;
        m_x.set(2, 0, theta);

        // Covariance update: P = (I - K) * P
        Matrix<N3, N3> I = Matrix.eye(Nat.N3());
        m_P = I.minus(K).times(m_P);

        // Note: We don't add process noise Q here because the robot is assumed
        // to be truly motionless. Adding Q would prevent convergence.

        m_measurementCount++;
    }

    /**
     * Gets the current pose estimate.
     *
     * @return The estimated pose, or Pose2d(0,0,0) if not initialized
     */
    public Pose2d getEstimate() {
        if (!m_initialized) {
            return new Pose2d();
        }
        return new Pose2d(
            m_x.get(0, 0),
            m_x.get(1, 0),
            new Rotation2d(m_x.get(2, 0))
        );
    }

    /**
     * Gets the current state covariance matrix.
     *
     * @return The 3x3 covariance matrix, or null if not initialized
     */
    public Matrix<N3, N3> getCovariance() {
        return m_P;
    }

    /**
     * Checks if the filter has been initialized with at least one measurement.
     *
     * @return true if initialized
     */
    public boolean isInitialized() {
        return m_initialized;
    }

    /**
     * Gets the number of measurements injected since last reset.
     *
     * @return measurement count
     */
    public int getMeasurementCount() {
        return m_measurementCount;
    }

    /**
     * Checks if the position estimate has converged to within specified thresholds.
     *
     * @param positionThresholdMeters Maximum acceptable position standard deviation (meters)
     * @param angleThresholdRadians Maximum acceptable angle standard deviation (radians)
     * @return true if converged (all uncertainties below thresholds)
     */
    public boolean hasConverged(double positionThresholdMeters, double angleThresholdRadians) {
        if (!m_initialized) {
            return false;
        }

        // Check diagonal elements of P (variances)
        // Variance must be below threshold^2 (since threshold is in std dev units)
        double posVarThreshold = positionThresholdMeters * positionThresholdMeters;
        double angleVarThreshold = angleThresholdRadians * angleThresholdRadians;

        return m_P.get(0, 0) < posVarThreshold
            && m_P.get(1, 1) < posVarThreshold
            && m_P.get(2, 2) < angleVarThreshold;
    }

    /**
     * Checks convergence using default thresholds from constants.
     *
     * @return true if converged
     */
    public boolean hasConverged() {
        return hasConverged(
            VisionKalmanConstants.kConvergencePositionThreshold,
            Math.toRadians(VisionKalmanConstants.kConvergenceAngleThresholdDegrees)
        );
    }

    /**
     * Returns the position standard deviation (average of x and y).
     *
     * @return Position std dev in meters, or Double.MAX_VALUE if not initialized
     */
    public double getPositionStdDev() {
        if (!m_initialized) {
            return Double.MAX_VALUE;
        }
        double xVar = m_P.get(0, 0);
        double yVar = m_P.get(1, 1);
        return Math.sqrt((xVar + yVar) / 2.0);
    }

    /**
     * Returns the angle standard deviation.
     *
     * @return Angle std dev in degrees, or Double.MAX_VALUE if not initialized
     */
    public double getAngleStdDevDegrees() {
        if (!m_initialized) {
            return Double.MAX_VALUE;
        }
        return Math.toDegrees(Math.sqrt(m_P.get(2, 2)));
    }

    /**
     * Gets display info for field visualization, applying a forward offset
     * to the estimated pose for visibility.
     *
     * @param forwardOffsetMeters Forward offset to apply to the pose (meters)
     * @return DisplayInfo with the offset pose and convergence status
     */
    public DisplayInfo getFieldDisplayInfo(double forwardOffsetMeters) {
        Optional<Pose2d> pose = m_initialized
            ? Optional.of(getEstimate()
                .transformBy(new Transform2d(forwardOffsetMeters, 0, new Rotation2d())))
            : Optional.empty();
        return new DisplayInfo(pose, hasConverged());
    }

    /**
     * Resets the filter, clearing all state and measurements.
     */
    public void reset() {
        m_initialized = false;
        m_x = null;
        m_P = null;
        m_measurementCount = 0;
    }

    /**
     * Creates the initial covariance matrix with high uncertainty.
     */
    private Matrix<N3, N3> createInitialCovariance() {
        Matrix<N3, N3> P = new Matrix<>(Nat.N3(), Nat.N3());
        P.set(0, 0, VisionKalmanConstants.kInitialPositionVariance);
        P.set(1, 1, VisionKalmanConstants.kInitialPositionVariance);
        P.set(2, 2, VisionKalmanConstants.kInitialAngleVariance);
        return P;
    }

    /**
     * Creates the process noise matrix Q.
     * Very small since we assume the robot is motionless.
     */
    private Matrix<N3, N3> createProcessNoise() {
        Matrix<N3, N3> Q = new Matrix<>(Nat.N3(), Nat.N3());
        Q.set(0, 0, VisionKalmanConstants.kProcessNoisePosition);
        Q.set(1, 1, VisionKalmanConstants.kProcessNoisePosition);
        Q.set(2, 2, VisionKalmanConstants.kProcessNoiseAngle);
        return Q;
    }

    /**
     * Computes measurement noise R based on tag count.
     * More tags = lower noise = more trust in measurement.
     *
     * @param tagCount Number of visible tags (must be >= 2)
     */
    private Matrix<N3, N3> computeMeasurementNoise(int tagCount) {
        // Scale base noise inversely with tag count
        double posNoise = VisionKalmanConstants.kBasePositionNoise / tagCount;
        double angleNoise = VisionKalmanConstants.kBaseAngleNoise / tagCount;

        Matrix<N3, N3> R = new Matrix<>(Nat.N3(), Nat.N3());
        R.set(0, 0, posNoise);
        R.set(1, 1, posNoise);
        R.set(2, 2, angleNoise);
        return R;
    }
}
