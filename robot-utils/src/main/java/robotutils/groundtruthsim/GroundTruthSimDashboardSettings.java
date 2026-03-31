package robotutils.groundtruthsim;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;

/** Field we want to display on the dashboard. */
public record GroundTruthSimDashboardSettings(
    Pose2d groundTruthPose,
    Pose2d estimatedPose,
    SwerveModuleState[] estimatedModuleStates,
    SwerveModulePosition[] estimatedModulePositions,
    double poseEstimateToGroundTruthDistance) {
}
