package frc.robot.visutils;

import java.util.Optional;

/**
 * Class to keep track of the best/first cameras
 * that had targets this cycle.
 */
@SuppressWarnings("MemberName")
public class PerCycleState {
    public Optional<SingleCamOdometry> bestLockedCam;
    public double bestLockedCamScore;

    public boolean isAnyCameraLockedOn;
    public boolean isAnyCameraMultiLockedOn;

    /** Constructor. */
    public PerCycleState() {
        reset();
    }

    /** Reset values. */
    public void reset() {
        bestLockedCam = Optional.empty();
        bestLockedCamScore = -1;

        isAnyCameraLockedOn = false;
        isAnyCameraMultiLockedOn = false;
    }
}
