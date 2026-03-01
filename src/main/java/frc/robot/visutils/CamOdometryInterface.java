package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

public interface CamOdometryInterface {
    Optional<Pose2d> getLatestVisPose();
    double getCurrentConfidenceScore();
    int getNumLockedTags();
    List<Integer> getTargetList();


    /**
     * Returns True if ANY camera currently has a lock on at-least one
     * target.
     */
    // $TODO - isAnyCameraLockedOn

    /**
     * Returns True if ANY camera has a lock on at least TWO targets,
     * i.e. a multi-lock.
     */
    // $TODO - isAnyCameraMultiLockedOn

    /**
     * Gets the ID of the target currently locked onto by
     * the PRIMARY camera, or -1 if no target is locked.
     * This method is generally used to tell what target the robot
     * is currently aligned to, so that the robot can turn towards it.
      */
    int getCurrentlyAlignedAprilTagId();

    /**
     * Get the horizontal offset from the currently locked onto
     * target for the PRIMARY camera.  Returns 0 if no target is locked.
     * This method is generally used to tell how far off the robot is from
     * being aligned to the target, so that the robot can turn towards it.
     */
    // $TODO - Rename to getCurrentlyAlignedTx
    double getTx();
}
