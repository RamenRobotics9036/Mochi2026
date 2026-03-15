package frc.robot.visutils.evaluateposes;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.util.MathUtils;
import java.util.OptionalDouble;


/** Mochi 2026 implementation of {@link EvaluatePosesInterface}. */
public class EvaluatePosesMochiV1 implements EvaluatePosesInterface {
    /** Maximum allowed ERROR (meters) between vision pose and current pose. */
    private static final double MAX_ERROR_METERS = 1.0;

    // The standard deviations of our vision estimated poses, which affect correction rate
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

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

        // Ambiguity only matters for MegaTag1 (pure visual PnP); MT2 resolves ambiguity
        // using the gyro heading, so this check must not apply to MT2 estimates.
        if (!poseEstimate.isMegaTag2
            && poseEstimate.tagCount == 1
            && poseEstimate.rawFiducials.length == 1) {
            if (poseEstimate.rawFiducials[0].ambiguity > 0.7) {
                return true;
            }
        }

        // Check if std devs indicate rejection
        if (curStdDevs.get(0, 0) == Double.MAX_VALUE) {
            return true;
        }

        return false;
    }

    @Override
    public PoseEstimate pickMegatag1vsMegatag2(PoseEstimate mt1, PoseEstimate mt2) {
        return pickBestEstimate(mt1, mt2);
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic std
     * deviations based on number of tags and distance from the tags.
     *
     * @param enhancedPoseEstimate The enhanced pose estimate to evaluate
     * @return The calculated standard deviations matrix
     */
    @Override
    public Matrix<N3, N1> calcVisionPoseStdDev(EnhancedPoseEstimate enhancedPoseEstimate) {
        PoseEstimate poseEstimate = enhancedPoseEstimate.getVisionPoseEstimate();
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
        // Single-tag poses become unreliable past 4 m regardless of MT1 or MT2 — the gyro
        // fusion in MT2 only resolves rotational ambiguity, not translational accuracy at
        // long range. Reject single-tag estimates beyond 4 m for both pipelines.
        if (numTags == 1 && avgDist > 4) {
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
     * Picks the better of two pose estimates. More tags wins; on a tie, closer
     * average tag distance wins (with MT2 getting a distance advantage since its
     * gyro-fused rotation is more reliable at similar range); if still tied,
     * prefer MT2 for gyro-fused stability. Either or both may be null.
     */
    static PoseEstimate pickBestEstimate(
        PoseEstimate a,
        PoseEstimate b) {

        if (a == null) {
            return b;
        }
        if (b == null) {
            return a;
        }
        if (a.tagCount != b.tagCount) {
            return a.tagCount > b.tagCount ? a : b;
        }
        // Give MT2 a distance advantage by penalizing non-MT2 estimates,
        // so values never go negative.
        double distA = a.avgTagDist
            + (a.isMegaTag2 ? 0.0 : VisionConstants.kMt2DistanceAdvantageMeter);
        double distB = b.avgTagDist
            + (b.isMegaTag2 ? 0.0 : VisionConstants.kMt2DistanceAdvantageMeter);

        if (!MathUtils.approxEqual(distA, distB)) {
            return distA < distB ? a : b;
        }
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
