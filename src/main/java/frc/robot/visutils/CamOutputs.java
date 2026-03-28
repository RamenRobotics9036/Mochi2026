package frc.robot.visutils;

import java.util.function.Consumer;
import robotutils.interfaces.VisionSimInterface.DrivetrainVisionPoseInfo;


/**
 * Destinations that {@link SingleCamOdometry} pushes results into:
 * the drivetrain's vision-measurement consumer and the Kalman filter.
 */
public record CamOutputs(
    Consumer<DrivetrainVisionPoseInfo> drivetrainVisionPoseConsumer,
    Consumer<DrivetrainVisionPoseInfo> visionKalmanMeasurementConsumer) {}
