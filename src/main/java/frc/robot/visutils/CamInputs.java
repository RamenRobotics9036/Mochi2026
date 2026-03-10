package frc.robot.visutils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Robot-state sources that {@link SingleCamOdometry} reads each cycle:
 * current drive state (for orientation/heading), a historical pose sampler
 * (for error-at-snap-time), and a motionless flag (for Kalman injection).
 */
public record CamInputs(
    Supplier<SwerveDriveState> driveStateSupplier,
    Function<Double, Optional<Pose2d>> robotPoseAtTimeSupplier,
    BooleanSupplier isMotionlessSupplier) {}
