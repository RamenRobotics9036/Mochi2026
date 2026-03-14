package frc.robot.visutils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.sim.visionproducers.VisionSimInterface;
import frc.robot.visutils.evaluateposes.EvaluatePosesInterface;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Shared dependencies for all {@link SingleCamOdometry} instances in a multi-camera setup.
 * Per-camera values (limelightName, robotToCam) are passed separately to each instance.
 */
public record CamOdometryDeps(
        VisionSimInterface.EstimateConsumer poseConsumer,
        Supplier<SwerveDriveState> driveStateSupplier,
        Function<Double, Optional<Pose2d>> poseSampler,
        boolean megaTag2Enabled,
        boolean autoVisionInjectionEnabled,
        EvaluatePosesInterface evaluatePoses,
        VisionKalmanFilter visionKalmanFilter,
        BooleanSupplier isMotionlessSupplier) {}
