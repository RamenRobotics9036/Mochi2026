package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.BotConfigInterface.CameraInfo;
import frc.robot.sim.visionproducers.VisionSimInterface;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

/** Factory for creating and wiring up a {@link MultiCamOdometry} instance. */
public class MultiCamOdometryFactory {

    /** Constructor. */
    private MultiCamOdometryFactory() {}

    /**
     * Creates a {@link MultiCamOdometry} and wires it to the dashboard and vision filter.
     *
     * @param configInterface    Bot configuration (cameras, speeds, etc.)
     * @param poseSampler        Samples the drivetrain's historical pose at a given timestamp
     *                           (e.g. {@code drivetrain::samplePoseAt})
     * @param poseConsumer       Consumer for vision pose estimates (e.g. {@code drivetrain::addVisionMeasurement})
     * @param yawDegreesSupplier Supplies the robot's current heading in degrees (WPILib blue-alliance
     *                           frame) for MegaTag2 orientation updates
     * @param basicInfoDashboard Dashboard to receive vision confidence/status updates
     * @param visionKalmanFilter Kalman filter for stationary vision estimation
     * @param motionlessTracker  Tracks whether the robot is motionless
     * @return A fully configured {@link CamOdometryInterface} instance
     */
    public static CamOdometryInterface create(
            BotConfigInterface configInterface,
            Function<Double, Optional<Pose2d>> poseSampler,
            VisionSimInterface.EstimateConsumer poseConsumer,
            DoubleSupplier yawDegreesSupplier,
            BasicInfoDashboard basicInfoDashboard,
            VisionKalmanFilter visionKalmanFilter,
            MotionlessTracker motionlessTracker) {

        List<CamOdometryInterface> cameras = new ArrayList<>();
        for (CameraInfo camInfo : configInterface.getCameras()) {
            cameras.add(new SingleCamOdometry(
                camInfo.cameraName,
                camInfo.robotToCam,
                poseConsumer,
                poseSampler,
                yawDegreesSupplier,
                VisionConstants.kSupportMegatag2));
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
            multiCam::isLatestMt2,
            multiCam::getPrimaryTagTx,
            multiCam::getVisibleTagIds,
            multiCam::getVisionErrorAtSnapTime,
            motionlessTracker::isMotionless,
            motionlessTracker::getSecondsStill);

        basicInfoDashboard.setVisionKalmanSupplier(() -> visionKalmanFilter);

        return multiCam;
    }
}
