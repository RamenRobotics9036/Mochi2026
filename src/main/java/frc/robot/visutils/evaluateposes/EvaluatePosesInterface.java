package frc.robot.visutils.evaluateposes;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.OptionalDouble;


/**
 * Defines the strategy for evaluating and selecting vision pose estimates.
 */
public interface EvaluatePosesInterface {

    /**
     * Should pose be rejected.
     */
    boolean isVisionPoseBad(
        EnhancedPoseEstimate enhancedPoseEstimate,
        Matrix<N3, N1> curStdDevs,
        double curConfidenceScore,
        SwerveDriveState driveState);

    /**
     * Pick best pose for single camera (either MT1 or MT2).
     * NOTE: This should NEVER return a pose that is not isVisionPoseGood().
     */
    PoseEstimate pickMegatag1vsMegatag2(PoseEstimate mt1, PoseEstimate mt2);

    /**
     * Calculates the (x, y, theta) standard-deviation matrix for a pose estimate.
     * Larger values indicate less trust in that axis.  Returning
     * {@code [Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE]} signals that the
     * estimate should be fully rejected by the pose estimator.
     */
    Matrix<N3, N1> calcVisionPoseStdDev(EnhancedPoseEstimate enhancedPoseEstimate);

}
