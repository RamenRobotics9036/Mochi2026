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

            // Is this the strongest lock we have seen so far?
            double singleCamScore = cam.getCurrentConfidenceScore();
            if (singleCamScore > m_perCycleState.bestLockedCamScore) {
                m_perCycleState.bestLockedCam = Optional.of(cam);
                m_perCycleState.bestLockedCamScore = singleCamScore;
            }

            if (cam.isAnyCameraLockedOn()) {
                m_perCycleState.isAnyCameraLockedOn = true;
            }

            if (cam.isAnyCameraMultiLockedOn()) {
                m_perCycleState.isAnyCameraMultiLockedOn = true;
            }
        }
    }

    /** Proxy calls to helper. */
    // $TODO - No proxies please
    public Optional<Pose2d> getLatestVisPoseForSingleCam(CameraSelectionMode selectionMode) {
        return m_consolidateResults.getLatestVisPoseForSingleCam(
            m_perCycleState,
            selectionMode);
    }

    /** Proxy calls to helper. */
    // $TODO - No proxies please
    public double getCurrentConfidenceScoreForSingleCam(CameraSelectionMode selectionMode) {
        return m_consolidateResults.getCurrentConfidenceScoreForSingleCam(
            m_perCycleState,
            selectionMode);
    }

    /** Proxy calls to helper. */
    // $TODO - No proxies please
    public List<Integer> getTargetListForAllCams() {
        return m_consolidateResults.getTargetListForAllCams();
    }

    // $TODO - I think this can move into MultiCamOdometryFactory.create
    public double getBestCurrentConfidenceScoreForSingleCam() {
        return getCurrentConfidenceScoreForSingleCam(
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
    public List<Integer> getTargetList() {
        return getTargetListForAllCams();
    }

    @Override
    public boolean isAnyCameraLockedOn() {
        return m_perCycleState.maxLockCount > 0;
    }

    @Override
    public boolean isAnyCameraMultiLockedOn() {
        return m_perCycleState.maxLockCount > 1;
    }

    @Override
    public double getCurrentlyAlignedTx() {
        // NOTE: Always returns Camera 0 forward-facing camera result, since
        // we use this to align robot rotationally to a target.
        return getPrimaryCam().getCurrentlyAlignedTx();
    }

    @Override
    public int getCurrentlyAlignedAprilTagId() {
        // NOTE: Always returns Camera 0 forward-facing camera result, since
        // we use this to align robot rotationally to a target.
        return getPrimaryCam().getCurrentlyAlignedAprilTagId();
    }

    private SingleCamOdometry getPrimaryCam() {
        // Just return the first cam in the list, which is hopefully
        // the forward-facing cam.
        return m_singleCamLimelightList.get(0);
    }
}
