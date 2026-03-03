package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.BotConfigInterface.CameraInfo;
import frc.robot.sim.visionproducers.VisionSimInterface;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;


/** Reads from multiple limelight cameras. */
public class MultiCamOdometry implements CamOdometryInterface {
    private final BotConfigInterface m_configInterface;
    private final List<SingleCamOdometry> m_singleCamLimelightList;
    private final PerCycleState m_perCycleState = new PerCycleState();

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

    @Override
    public Optional<Pose2d> getLatestVisPose() {
        // NOTE: We return the pose from the camera with the strongest lock, if it exists.
        if (m_perCycleState.bestLockedCam.isPresent()) {
            return m_perCycleState.bestLockedCam.get().getLatestVisPose();
        }
        else {
            return Optional.empty();
        }
    }

    @Override
    public double getCurrentConfidenceScore() {
        // NOTE: We return the confidence score from the camera with the strongest lock, if it exists.
        if (m_perCycleState.bestLockedCam.isPresent()) {
            return m_perCycleState.bestLockedCam.get().getCurrentConfidenceScore();
        }
        else {
            return 0.0;
        }
    }

    @Override
    public List<Integer> getTargetList() {
        TreeSet<Integer> seen = new TreeSet<>();
        for (SingleCamOdometry cam : m_singleCamLimelightList) {
            seen.addAll(cam.getTargetList());
        }
        return new ArrayList<>(seen);
    }

    @Override
    public boolean isAnyCameraLockedOn() {
        return m_perCycleState.isAnyCameraLockedOn;
    }

    @Override
    public boolean isAnyCameraMultiLockedOn() {
        return m_perCycleState.isAnyCameraMultiLockedOn;
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
