package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Per;
import frc.robot.visutils.PerCycleState.CameraSelectionMode;
import java.util.ArrayList;
import java.util.List;
import java.util.TreeSet;
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
    public Optional<Pose2d> getLatestVisPoseForSingleCam(
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

    /**
     * For the current cycle, get the confidence score from one particular camera.
     * Returns 0.0 if no camera has a lock.
     */
    public double getCurrentConfidenceScoreForSingleCam(
        PerCycleState cyclestate,
        CameraSelectionMode selectionMode) {

        Optional<SingleCamOdometry> cam = getAppropriateCamera(
            cyclestate,
            selectionMode);

        if (cam.isPresent()) {
            return cam.get().getCurrentConfidenceScore();
        }

        return 0.0;
    }

    /**
     * Returns the total number of AprilTags locked-on for a SINGLE camera
     * (e.g. best camera, or cam0).
     */
    public int getNumLockedTagsForSingleCam(
        PerCycleState cyclestate,
        CameraSelectionMode selectionMode) {

        Optional<SingleCamOdometry> cam = getAppropriateCamera(
            cyclestate,
            selectionMode);

        if (cam.isPresent()) {
            return cam.get().getNumLockedTags();
        }

        return 0;
    }

    /**
     * Returns Tx for a SINGLE camera (e.g. best camera, or cam0).
     * Returns 0.0 if no camera has a lock.
     */
    public double getTxForSingleCam(
        PerCycleState cyclestate,
        CameraSelectionMode selectionMode) {

        Optional<SingleCamOdometry> cam = getAppropriateCamera(
            cyclestate,
            selectionMode);

        if (cam.isPresent()) {
            return cam.get().getTx();
        }

        return 0.0;
    }

    /**
     * Returns a list of UNIQUE integers representing the AprilTags that are
     * currently locked for ALL cameras.
     */
    public List<Integer> getTargetListForAllCams() {
        TreeSet<Integer> seen = new TreeSet<>();
        for (SingleCamOdometry cam : m_singleCamOdomList) {
            seen.addAll(cam.getTargetList());
        }
        return new ArrayList<>(seen);
    }

    /**
     * Returns the ID of the last seen fiducial, or -1 if none.
     */
    public int getLastTargetForSingleCam(
        PerCycleState cyclestate,
        CameraSelectionMode selectionMode) {

        Optional<SingleCamOdometry> cam = getAppropriateCamera(
            cyclestate,
            selectionMode);

        if (cam.isPresent()) {
            return cam.get().getLastTarget();
        }

        return -1;
    }
}
