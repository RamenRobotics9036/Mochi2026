package robotutils.groundtruthsim;

import edu.wpi.first.math.geometry.Pose2d;

/** Field we want to display on the dashboard. */
public record GroundTruthSimDashboardSettings(Pose2d groundTruthPose) {
}