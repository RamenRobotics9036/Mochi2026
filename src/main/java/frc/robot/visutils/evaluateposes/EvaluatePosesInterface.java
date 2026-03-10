package frc.robot.visutils.evaluateposes;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.List;
import java.util.Optional;


/**
 * Defines the strategy for evaluating and selecting vision pose estimates.
 */
public interface EvaluatePosesInterface {

    /**
     * Should pose be rejected.
     */
    boolean isVisionPoseBad(PoseEstimate poseEstimate, Pose2d currentRobotPose);

    /**
     * Pick best pose for single camera (either MT1 or MT2).
     * NOTE: This should NEVER return a pose that is not isVisionPoseGood().
     */
    PoseEstimate pickMegatag1vsMegatag2(PoseEstimate mt1, PoseEstimate mt2); // $TODO2 - Don't forget to check return-value is isVisionPoseGood()

    /**
     * Given 1 pose from each of N cameras, pick the best one.
     * NOTE: This should NEVER return a pose that is not isVisionPoseGood().
     */
    Optional<PoseEstimate> multiCamPickBestPose(List<PoseEstimate> poseEstimateList); // $TODO2 - Don't forget to check return-value is isVisionPoseGood()

    /**
     * Given 1 pose from each of N cameras, picks ALL poses that should be injected
     * into drivetrain Kalman filter.
     * NOTE: This should NEVER return a pose that is not isVisionPoseGood().
     */
    List<PoseEstimate> multiCamPickAllPosesToInject(List<PoseEstimate> poseEstimateList); // $TODO2 - Don't forget to check return-value is isVisionPoseGood()

    /**
     * Calculates the (x, y, theta) standard-deviation matrix for a pose estimate.
     * Larger values indicate less trust in that axis.  Returning
     * {@code [Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE]} signals that the
     * estimate should be fully rejected by the pose estimator.
     */
    Matrix<N3, N1> calcVisionPoseStdDev(PoseEstimate poseEstimate);

    /**
     * Calculates a simple score in [0, 100] for a pose.
     * A score of 0 means no confidence (estimate should be ignored); 100 means maximum
     * confidence.
     */
    double calcVisionPoseScore(Matrix<N3, N1> stdDevs);
}
