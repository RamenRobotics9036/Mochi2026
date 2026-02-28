package frc.robot.visutils;

import frc.robot.botconfig.BotConfigInterface;
import frc.robot.sim.visionproducers.VisionSimInterface;

/** Factory for creating and wiring up a {@link MultiCamOdometry} instance. */
public class MultiCamOdometryFactory {

    private MultiCamOdometryFactory() {}

    /**
     * Creates a {@link MultiCamOdometry} and wires it to the dashboard and vision filter.
     *
     * @param configInterface    Bot configuration (cameras, speeds, etc.)
     * @param poseConsumer       Consumer for vision pose estimates (e.g. {@code drivetrain::addVisionMeasurement})
     * @param basicInfoDashboard Dashboard to receive vision confidence/status updates
     * @param visionKalmanFilter Kalman filter for stationary vision estimation
     * @param motionlessTracker  Tracks whether the robot is motionless
     * @return A fully configured {@link MultiCamOdometry} instance
     */
    public static MultiCamOdometry create(
            BotConfigInterface configInterface,
            VisionSimInterface.EstimateConsumer poseConsumer,
            BasicInfoDashboard basicInfoDashboard,
            VisionKalmanFilter visionKalmanFilter,
            MotionlessTracker motionlessTracker) {

        MultiCamOdometry multiCam = new MultiCamOdometry(
            configInterface,
            poseConsumer);

        multiCam.setVisionDependencies(
            basicInfoDashboard::isVisionEnabled,
            visionKalmanFilter,
            motionlessTracker::isMotionless);

        return multiCam;
    }
}
