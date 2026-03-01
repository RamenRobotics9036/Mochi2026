package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.BotConfigInterface.CameraInfo;
import frc.robot.sim.visionproducers.VisionSimInterface;
import frc.robot.visutils.PerCycleState.CameraSelectionMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;


/** Reads from multiple limelight cameras. */
public class MultiCamOdometry implements CamOdometryInterface {
    private final BotConfigInterface m_configInterface;
    private final List<SingleCamOdometry> m_singleCamLimelightList;
    private final PerCycleState m_perCycleState = new PerCycleState();
    private final ConsolidateMultipleCamResults m_consolidateResults;

    /** Constructor. */
    public MultiCamOdometry(
        BotConfigInterface configInterface,
        VisionSimInterface.EstimateConsumer poseConsumer) {

        m_configInterface = configInterface;

        List<CameraInfo> cameraInfoList = m_configInterface.getCameras();

        m_singleCamLimelightList = new ArrayList<>();
        for (CameraInfo camInfo : cameraInfoList) {
            m_singleCamLimelightList.add(new SingleCamOdometry(
                camInfo.cameraName,
                camInfo.robotToCam,
                poseConsumer));
        }

        m_consolidateResults = new ConsolidateMultipleCamResults(
            m_singleCamLimelightList);
    }

    /**
     * Sets the dependencies needed for vision processing.
     *
     * @param visionEnabledSupplier A BooleanSupplier returning true when vision is enabled
     * @param filter The VisionKalmanFilter instance to inject measurements into
     * @param isMotionlessSupplier Supplier that returns true when robot is motionless
     */
    public void setVisionDependencies(
            BooleanSupplier visionEnabledSupplier,
            VisionKalmanFilter filter,
            BooleanSupplier isMotionlessSupplier) {

        for (SingleCamOdometry cam : m_singleCamLimelightList) {
            cam.setVisionDependencies(
                visionEnabledSupplier,
                filter,
                isMotionlessSupplier);
        }
    }

    /**
     * Periodic update; should be called from robot periodic.
     * NOTE: For each cycle, track the camera with thehighest confidence
     *   score that has a target lock.  We also track the FIRST camera
     *   (in order) that has a target lock.
     */
    public void periodic() {
        // Reset best locked cam
        m_perCycleState.reset();

        for (SingleCamOdometry cam : m_singleCamLimelightList) {
            cam.periodic();

            double singleCamScore = cam.getCurrentConfidenceScore();
            boolean camHasTargets = cam.getNumLockedTags() > 0;
            if (camHasTargets) {
                // If it's first cam in list with a lock, remember that
                if (m_perCycleState.firstInOrderLockedCam.isEmpty()) {
                    m_perCycleState.firstInOrderLockedCam = Optional.of(cam);
                }

                // Is this the strongest lock we have seen so far?
                if (singleCamScore > m_perCycleState.bestLockedCamScore) {
                    m_perCycleState.bestLockedCam = Optional.of(cam);
                    m_perCycleState.bestLockedCamScore = singleCamScore;
                }
            }
        }
    }

    /** Proxy calls to helper. */
    public Optional<Pose2d> getLatestVisPoseForSingleCam(CameraSelectionMode selectionMode) {
        return m_consolidateResults.getLatestVisPoseForSingleCam(
            m_perCycleState,
            selectionMode);
    }

    /** Proxy calls to helper. */
    public double getCurrentConfidenceScoreForSingleCam(CameraSelectionMode selectionMode) {
        return m_consolidateResults.getCurrentConfidenceScoreForSingleCam(
            m_perCycleState,
            selectionMode);
    }

    /** Proxy calls to helper. */
    public int getNumLockedTagsForSingleCam(CameraSelectionMode selectionMode) {
        return m_consolidateResults.getNumLockedTagsForSingleCam(
            m_perCycleState,
            selectionMode);
    }

    /** Proxy calls to helper. */
    public double getTxForSingleCam(CameraSelectionMode selectionMode) {
        return m_consolidateResults.getTxForSingleCam(
            m_perCycleState,
            selectionMode);
    }

    /** Proxy calls to helper. */
    public List<Integer> getTargetListForAllCams() {
        return m_consolidateResults.getTargetListForAllCams();
    }

    // $TODO - I think this can move into MultiCamOdometryFactory.create
    public double getBestCurrentConfidenceScoreForSingleCam() {
        return getCurrentConfidenceScoreForSingleCam(
            CameraSelectionMode.CAMERA_BEST_WITH_LOCK);
    }

    // $TODO - I think this can move into MultiCamOdometryFactory.create
    public int getBestNumLockedTagsForSingleCam() {
        return getNumLockedTagsForSingleCam(
            CameraSelectionMode.CAMERA_BEST_WITH_LOCK);
    }

    // $TODO - I think this can move into MultiCamOdometryFactory.create
    public double getBestTxForSingleCam() {
        return getTxForSingleCam(
            CameraSelectionMode.CAMERA_BEST_WITH_LOCK);
    }

    @Override
    public Optional<Pose2d> getLatestVisPose() {
        return getLatestVisPoseForSingleCam(CameraSelectionMode.CAMERA_BEST_WITH_LOCK);
    }

    @Override
    public double getCurrentConfidenceScore() {
        return getCurrentConfidenceScoreForSingleCam(CameraSelectionMode.CAMERA_BEST_WITH_LOCK);
    }

    @Override
    public int getNumLockedTags() {
        return getNumLockedTagsForSingleCam(CameraSelectionMode.CAMERA_BEST_WITH_LOCK);
    }

    @Override
    public double getTx() {
        return getTxForSingleCam(CameraSelectionMode.CAMERA_BEST_WITH_LOCK);
    }

    @Override
    public List<Integer> getTargetList() {
        return getTargetListForAllCams();
    }

    @Override
    public int getCurrentlyAlignedAprilTagId() {
        // NOTE: Always returns Camera 0 forward-facing camera result, since
        // we use this to align robot rotationally to a target.
        return m_consolidateResults.getCurrentlyAlignedAprilTagId(
            m_perCycleState,
            CameraSelectionMode.CAMERA_ALWAYS_CAM0);
    }
}
