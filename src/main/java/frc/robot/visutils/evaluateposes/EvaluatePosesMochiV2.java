package frc.robot.visutils.evaluateposes;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.util.MathUtils;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

/** Mochi 2026 implementation of {@link EvaluatePosesInterface}. */
public class EvaluatePosesMochiV2 implements EvaluatePosesInterface {
    /** Maximum allowed ERROR (meters) between vision pose and current pose */
    private static final double MAX_ERROR_METERS = 1.0;

    // MT1 is configured to be effectively ignored for X/Y position (very large
    // stddev) while still being trusted for rotation. The 1e6 X/Y values indicate
    // extremely high uncertainty in translation so pose estimators will down‑weight
    // MT1's position contribution, but the relatively small rotational stddev (~3
    // degrees) allows MT1 to meaningfully contribute to heading estimation.
    public static final Matrix<N3, N1> MT1_STDDEV = VecBuilder.fill(1e6, 1e6, Math.PI / 60);
    // MT2 is the complementary measurement source: it is trusted for X/Y
    // translation (small stddevs) and effectively ignored for rotation (very large
    // stddev). Together, these settings implement "use only x/y from MT2" and "use
    // only rotation from MT1" when fusing measurements.
    public static final Matrix<N3, N1> MT2_STDDEV = VecBuilder.fill(0.5, 0.5, 1e6);

    @Override
    public boolean isVisionPoseBad(
        PoseEstimate poseEstimate,
        Matrix<N3, N1> curStdDevs,
        double curConfidenceScore,
        OptionalDouble errorAtSnapTime,
        SwerveDriveState driveState) {

        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            return true;
        }

        if (shouldIgnoreFarError(poseEstimate.pose, driveState.Pose)) {
            return true;
        }

        // If MT2 the pose is present
        if (poseEstimate.isMegaTag2) {
            // If we see >0 tags and robot rotates <2 rotations per second
            if (poseEstimate.tagCount > 0
                && Math.abs(Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond)) < 2) {

                // Add it to the pose estimator.
                return false;
            }
        }

        // Bad
        return true;
    }

    @Override
    public PoseEstimate pickMegatag1vsMegatag2(PoseEstimate mt1, PoseEstimate mt2) {
        return pickBestEstimate(mt1, mt2);
    }

    @Override
    public Optional<PoseEstimate> multiCamPickBestPose(List<PoseEstimate> poseEstimateList) {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public List<PoseEstimate> multiCamPickAllPosesToInject(List<PoseEstimate> poseEstimateList) {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic std
     * deviations based on number of tags and distance from the tags.
     *
     * @param poseEstimate The Limelight pose estimate to evaluate
     * @return The calculated standard deviations matrix
     */
    @Override
    public Matrix<N3, N1> calcVisionPoseStdDev(PoseEstimate poseEstimate) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            // Return very high std devs to indicate rejection of the pose estimate
            return VecBuilder.fill(1e6, 1e6, 1e6);
        }

        if (poseEstimate.isMegaTag2) {
            return getEstimationStdDevsLimelightMT2(poseEstimate, false);
        }
        else {
            return getEstimationStdDevsLimelightMT1(poseEstimate, false);
        }
    }

    /**
     * Retrieve estimated standard deviations for a Megatag 1 estimate
     * Leveraging code from:
     * https://github.com/PaisWillie/FRC-Rebuilt-2026/blob/f796a5feee722fe42ebfac3586a9ce2336e4bc12/src/main/java/frc/robot/utils/LimelightWrapper.java#L39
     *
     * @param poseEstimate the pose estimate from the limelight
     * @return the estimated standard deviations
     */
    private Matrix<N3, N1> getEstimationStdDevsLimelightMT1(
        PoseEstimate poseEstimate,
        boolean isLL4) {

        // Reject all limelight MT1
        return VecBuilder.fill(1e6, 1e6, 1e6);
    }

    /**
     * Retrieve estimated standard deviations for a Megatag 2 estimate
     *
     * @param poseEstimate the pose estimate from the limelight
     * @return the estimated standard deviations
     */
    private Matrix<N3, N1> getEstimationStdDevsLimelightMT2(
        PoseEstimate poseEstimate,
        boolean isLL4) {

        var estStdDevs = MT2_STDDEV;
        double stddevScalar = 1;

        int numTags = 0;
        double avgDist = 0;
        for (var value : poseEstimate.rawFiducials) {
            numTags++;
            avgDist += value.distToCamera;
        }

        // if no tags detected, ignorse the pose by returning very high std devs
        if (numTags == 0) {
            return VecBuilder.fill(1e6, 1e6, 1e6);
        }

        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            stddevScalar *= (0.65);
        }

        // Decrease std devs if limelight is LL4
        if (isLL4) {
            stddevScalar *= (.8);
        }

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 5) {
            estStdDevs = VecBuilder.fill(1e6, 1e6, 1e6);
        } else {
            stddevScalar *= (1 + (avgDist * avgDist * .2));
        }

        // apply the calculated scalar to the standard deviations
        estStdDevs = estStdDevs.times(stddevScalar);

        return estStdDevs;
    }

    /**
     * Converts std devs to a 0-100 confidence score.
     *
     * @param stdDevs The (x, y, theta) standard deviations matrix
     * @return Confidence from 0 (no confidence) to 100 (highest)
     */
    @Override
    public double calcVisionPoseScore(Matrix<N3, N1> stdDevs) {
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

    /**
     * Picks the better of two pose estimates. More tags wins; on a tie, closer
     * average tag distance wins (with MT2 getting a distance advantage since its
     * gyro-fused rotation is more reliable at similar range); if still tied,
     * prefer MT2 for gyro-fused stability. Either or both may be null.
     */
    static PoseEstimate pickBestEstimate(
        PoseEstimate a,
        PoseEstimate b) {

        if (a == null) return b;
        if (b == null) return a;

        // All else equal, prefer MT2 for gyro-fused rotation stability.
        return b.isMegaTag2 ? b : a;
    }

    private boolean shouldIgnoreFarError(Pose2d pose1, Pose2d pose2) {
        // Can't calculate distance with null poses
        if (pose1 == null || pose2 == null) {
            return true;
        }

        double distanceMeters = pose1.getTranslation().getDistance(pose2.getTranslation());
        return distanceMeters > MAX_ERROR_METERS;
    }
}
