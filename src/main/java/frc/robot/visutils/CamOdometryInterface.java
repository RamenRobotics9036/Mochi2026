package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

public interface CamOdometryInterface {
    Optional<Pose2d> getLatestVisPose();
    double getCurrentConfidenceScore();
    int getNumLockedTags();
    double getTx();
    List<Integer> getTargetList();
    int getLastTarget();
}
