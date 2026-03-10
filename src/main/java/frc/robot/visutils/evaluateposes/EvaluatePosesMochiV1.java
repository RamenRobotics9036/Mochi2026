package frc.robot.visutils.evaluateposes;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.util.MathUtils;

import java.util.List;
import java.util.Optional;

/** Mochi 2026 implementation of {@link EvaluatePosesInterface}. */
public class EvaluatePosesMochiV1 implements EvaluatePosesInterface {
    /** Maximum allowed ERROR (meters) between vision pose and current pose */
    private static final double MAX_ERROR_METERS = 1.0;

    @Override
    public boolean isVisionPoseBad(PoseEstimate poseEstimate, Pose2d currentRobotPose) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            return true;
        }

        return shouldIgnoreFarError(poseEstimate.pose, currentRobotPose);
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

    @Override
    public Matrix<N3, N1> calcVisionPoseStdDev(PoseEstimate poseEstimate) {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double calcVisionPoseScore(Matrix<N3, N1> stdDevs) {
        throw new UnsupportedOperationException("Not yet implemented");
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
        if (a.tagCount != b.tagCount) {
            return a.tagCount > b.tagCount ? a : b;
        }
        // Give MT2 a distance advantage by penalizing non-MT2 estimates, so values never go negative.
        double distA = a.avgTagDist + (a.isMegaTag2 ? 0.0 : VisionConstants.kMt2DistanceAdvantageMeter);
        double distB = b.avgTagDist + (b.isMegaTag2 ? 0.0 : VisionConstants.kMt2DistanceAdvantageMeter);
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
