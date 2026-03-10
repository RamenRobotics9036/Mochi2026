package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.BotConfigInterface.CameraInfo;
import frc.robot.sim.visionproducers.VisionSimInterface;
import frc.robot.visutils.evaluateposes.EvaluatePosesFactory;
import frc.robot.visutils.evaluateposes.EvaluatePosesInterface;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

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
     * @param poseConsumer       Consumer for vision pose estimates
     *                           (e.g. {@code drivetrain::addVisionMeasurement})
     * @param yawDegreesSupplier Supplies the robot's current heading in degrees
     *                           (WPILib blue-alliance frame) for MegaTag2 orientation updates
     * @param basicInfoDashboard Dashboard to receive vision confidence/status updates
     * @param visionKalmanFilter Kalman filter for stationary vision estimation
     * @param motionlessTracker  Tracks whether the robot is motionless
     * @param evaluatePosesName  Name of the {@link EvaluatePosesInterface} implementation
     *                           to use (e.g. {@code "MochiV1"})
     * @return A fully configured {@link CamOdometryInterface} instance
     */
    public static CamOdometryInterface create(
            BotConfigInterface configInterface,
            Function<Double, Optional<Pose2d>> poseSampler,
            Supplier<Pose2d> currentRobotPoseSupplier,
            VisionSimInterface.EstimateConsumer poseConsumer,
            DoubleSupplier yawDegreesSupplier,
            BasicInfoDashboard basicInfoDashboard,
            VisionKalmanFilter visionKalmanFilter,
            MotionlessTracker motionlessTracker,
            String evaluatePosesName) {

        boolean megaTag2Enabled = configInterface.isMegaTag2Supported();
        boolean autoVisionInjectionEnabled = configInterface.isAutoVisionInjectionEnabled();
        EvaluatePosesInterface evaluatePoses = EvaluatePosesFactory.create(evaluatePosesName);

        List<CamOdometryInterface> cameras = new ArrayList<>();
        for (CameraInfo camInfo : configInterface.getCameras()) {
            cameras.add(new SingleCamOdometry(
                camInfo.cameraName,
                camInfo.robotToCam,
                poseConsumer,
                currentRobotPoseSupplier,
                poseSampler,
                yawDegreesSupplier,
                megaTag2Enabled,
                autoVisionInjectionEnabled,
                evaluatePoses));
        }

        CamOdometryInterface multiCam = new MultiCamOdometry(
            cameras,
            megaTag2Enabled,
            autoVisionInjectionEnabled,
            evaluatePoses);

        MultiCamOdometryWrapper wrapper =
                new MultiCamOdometryWrapper(multiCam, configInterface.isVisionEnabledDefault());

        wrapper.setVisionDependenciesOnCamera(
            visionKalmanFilter,
            motionlessTracker::isMotionless);

        basicInfoDashboard.setVisionDependenciesOnDash(
            wrapper::getConfidenceScore,
            wrapper::hasTargetLock,
            wrapper::hasMultiTagLock,
            wrapper::isLatestMt2,
            wrapper::getPrimaryTagTx,
            wrapper::getVisibleTagIds,
            wrapper::getVisionErrorAtSnapTime,
            motionlessTracker::isMotionless,
            motionlessTracker::getSecondsStill);

        basicInfoDashboard.setVisionKalmanSupplier(() -> visionKalmanFilter);

        return wrapper;
    }
}
