package frc.robot.visutils;

import java.util.Optional;

/**
 * Class to keep track of the best/first cameras
 * that had targets this cycle.
 */
@SuppressWarnings("MemberName")
public class PerCycleState {
    // $TODO - CameraSelectionMode is not a good solution, remove it
    public enum CameraSelectionMode {
        CAMERA_ALWAYS_CAM0,
        CAMERA_FIRST_IN_ORDER_WITH_LOCK,
        CAMERA_BEST_WITH_LOCK
    }

    public Optional<SingleCamOdometry> bestLockedCam;
    public double bestLockedCamScore;
    public Optional<SingleCamOdometry> firstInOrderLockedCam;

    /** Constructor. */
    public PerCycleState() {
        reset();
    }

    /** Reset values. */
    public void reset() {
        bestLockedCam = Optional.empty();
        bestLockedCamScore = -1;
        firstInOrderLockedCam = Optional.empty();
    }
}
