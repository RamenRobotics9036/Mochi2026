package frc.robot.visutils;

import frc.robot.sim.visionproducers.VisionSimInterface.DrivetrainVisionPoseInfo;
import java.util.function.Consumer;


/**
 * Destinations that {@link SingleCamOdometry} pushes results into:
 * the drivetrain's vision-measurement consumer and the Kalman filter.
 */
public record CamOutputs(
    Consumer<DrivetrainVisionPoseInfo> drivetrainVisionPoseConsumer,
    Consumer<DrivetrainVisionPoseInfo> visionKalmanMeasurementConsumer) {}
