package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

public interface CamOdometryInterface {
    /**
     * Get pose estimate from the camera with highest confidence score.
     */
    Optional<Pose2d> getEstimatedPose();

    /**
     * Get confidence score of the camera with highest confidence score.
     * Returns 0.0 if no camera has a lock.
     */
    double getConfidenceScore();

    /**
     * Returns unique list of IDs of all AprilTags seen by any camera.
     */
    List<Integer> getVisibleTagIds();

    /**
     * Returns true if ANY camera currently has a lock on at least one target.
     */
    boolean hasTargetLock();

    /**
     * Returns true if ANY camera has a lock on at least TWO targets.
     */
    boolean hasMultiTagLock();

    /**
     * Gets the AprilTag ID locked onto by the PRIMARY camera, or -1 if none.
     * Used to determine which tag the robot is aligned to for turning.
     */
    int getPrimaryTagId();

    /**
     * Gets the horizontal offset (tx) to the primary tag for the PRIMARY camera.
     * Returns 0 if no target is locked. Used to determine rotation error for alignment.
     */
    double getPrimaryTagTx();
}
