package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.visutils.PerCycleState.CameraSelectionMode;
import java.util.List;
import java.util.Optional;

/** Given Vision results from multiple cameras, consolidates results. */
public class ConsolidateMultipleCamResults {
    private List<SingleCamOdometry> m_singleCamOdomList;

    /** Constructor. */
    public ConsolidateMultipleCamResults(
        List<SingleCamOdometry> singleCamOdomList) {

        m_singleCamOdomList = singleCamOdomList;
    }

    private Optional<SingleCamOdometry> getAppropriateCamera(
        PerCycleState cyclestate,
        CameraSelectionMode selectionMode) {

        switch (selectionMode) {
            case CAMERA_ALWAYS_CAM0:
                return Optional.of(m_singleCamOdomList.get(0));
            case CAMERA_FIRST_IN_ORDER_WITH_LOCK:
                return cyclestate.firstInOrderLockedCam;
            case CAMERA_BEST_WITH_LOCK:
                return cyclestate.bestLockedCam;
            default:
                throw new IllegalStateException("Unexpected value: " + selectionMode);
        }
    }

    /**
     * For the current cycle, gets the pose from one particular camera.
     * Returns empty if no pose.
     */
    public Optional<Pose2d> getLatestVisPose(
        PerCycleState cyclestate,
        CameraSelectionMode selectionMode) {

        Optional<SingleCamOdometry> cam = getAppropriateCamera(
            cyclestate,
            selectionMode);

        if (cam.isPresent()) {
            return cam.get().getLatestVisPose();
        }

        return Optional.empty();
    }
}
