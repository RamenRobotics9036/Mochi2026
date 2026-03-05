package frc.robot.visutils;

import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.BotConfigInterface.CameraInfo;
import frc.robot.sim.visionproducers.VisionSimInterface;
import java.util.ArrayList;
import java.util.List;

/** Factory for creating and wiring up a {@link MultiCamOdometry} instance. */
public class MultiCamOdometryFactory {

    /** Constructor. */
    private MultiCamOdometryFactory() {}

    /**
     * Creates a {@link MultiCamOdometry} and wires it to the dashboard and vision filter.
     *
     * @param configInterface    Bot configuration (cameras, speeds, etc.)
     * @param poseConsumer       Consumer for vision pose estimates (e.g. {@code drivetrain::addVisionMeasurement})
     * @param basicInfoDashboard Dashboard to receive vision confidence/status updates
     * @param visionKalmanFilter Kalman filter for stationary vision estimation
     * @param motionlessTracker  Tracks whether the robot is motionless
     * @return A fully configured {@link CamOdometryInterface} instance
     */
    public static CamOdometryInterface create(
            BotConfigInterface configInterface,
            VisionSimInterface.EstimateConsumer poseConsumer,
            BasicInfoDashboard basicInfoDashboard,
            VisionKalmanFilter visionKalmanFilter,
            MotionlessTracker motionlessTracker) {

        String finalstr = "";

        List<CamOdometryInterface> cameras = new ArrayList<>();
        for (CameraInfo camInfo : configInterface.getCameras()) {
            String str = "**ADDING CAMERA: " + camInfo.cameraName;
            finalstr = finalstr + str;
            cameras.add(new SingleCamOdometry(
                camInfo.cameraName,
                camInfo.robotToCam,
                poseConsumer));
        }

        CamOdometryInterface multiCam = new MultiCamOdometry(cameras);

        multiCam.setVisionDependencies(
            basicInfoDashboard::isVisionEnabled,
            visionKalmanFilter,
            motionlessTracker::isMotionless);

        basicInfoDashboard.setVisionDependencies(
            multiCam::getConfidenceScore,
            multiCam::hasTargetLock,
            multiCam::hasMultiTagLock,
            multiCam::getPrimaryTagTx,
            multiCam::getVisibleTagIds,
            motionlessTracker::isMotionless,
            motionlessTracker::getSecondsStill);

        basicInfoDashboard.setVisionKalmanSupplier(() -> visionKalmanFilter);

        return multiCam;
    }
}
