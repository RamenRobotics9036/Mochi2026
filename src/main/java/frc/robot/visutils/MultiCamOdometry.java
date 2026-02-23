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
import java.util.function.BooleanSupplier;


/** Reads from multiple limelight cameras. */
public class MultiCamOdometry {
    private final BotConfigInterface m_configInterface;
    private List<SingleCamOdometry> m_singleCamLimelightList;

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

    /** Periodic update; should be called from robot periodic. */
    public void periodic() {
        for (SingleCamOdometry cam : m_singleCamLimelightList) {
            cam.periodic();
        }
    }

    public Optional<Pose2d> getLatestVisPose() {
        // $TODO - For all these methods, we should either return the single
        // pose for this cycle with the highest confidence score, or
        // return a weighted average of the poses from each camera based on confidence scores.
        return m_singleCamLimelightList.get(0).getLatestVisPose();
    }

    public double getCurrentConfidenceScore() {
        return m_singleCamLimelightList.get(0).getCurrentConfidenceScore();
    }

    public int getNumLockedTags() {
        return m_singleCamLimelightList.get(0).getNumLockedTags();
    }

    public double getTx() {
        return m_singleCamLimelightList.get(0).getTx();
    }

    public String getTargetList() {
        // $TODO - This should probably be an aggregated list of targets from all cameras,
        // perhaps sorted by confidence score.
        return m_singleCamLimelightList.get(0).getTargetList();
    }

    /** Returns the ID of the last seen fiducial, or -1 if none. */
    public int getLastTarget() {
        // $TODO - Make sure the kalman filter locks onto the target from the camera
        // with the HIGHEST confidence score.  I don't think thats happening here,
        // so it may end up locking onto a random camera.
        return m_singleCamLimelightList.get(0).getLastTarget();
    }
}
