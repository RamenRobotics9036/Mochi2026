// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for team-wide values.
 * This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not use this class for utility methods.
 */
public final class Constants {

    private Constants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Constants for the Intake subsystem rollers.
     */
    public static final class IntakeConstants {
        /** CAN ID for the intake SPARK MAX motor controller. */
        public static final int kIntakeMotorID = 30;
        /** Default speed for pulling game pieces into the robot. */
        public static final double kIntakeSpeed = 0.8;
        /** Default speed for ejecting game pieces from the robot. */
        public static final double kOuttakeSpeed = -0.8;
        /** Current threshold in Amps used to detect if a game piece is fully secured. */
        public static final int kStallLimit = 40; // Amps
    }

    /**
     * Constants for the Intake subsystem rollers.
     */
    public static final class ShooterConstants {
        /** CAN ID for the left SparkFlex motor controller. */
        public static final int kLMotorID = 40;
        /** CAN ID for the right SparkFlex motor controller. */
        public static final int kRMotorID = 41;
        /** SmartCurrentLimit in amps*/
        public static final int kCurrentLimit = 40; //TODO: filler value
        /** PWM channer for the linear actuator servo. */
        public static final int kChannel = 0; //TODO: filler value
        /** Shooting speed */
        public static final double kShootSpeed = 0.20; //TODO: filler value
    }

    public static final class IndexerConstants {
        public static final int kMotorID = 30; //TODO: filler value
        /** Indexer flywheel speed */
        public static final double kIndexSpeed = 0.15; //TODO: filler value
    }

    /**
     * Configuration constants for the Operator Interface and telemetry.
     */
    public static final class OperatorConstants {
        /** When true, non-essential telemetry is disabled to conserve CAN bus bandwidth. */
        public static final boolean kCompetitionMode = false;
    }

    /**
     * Tuning constants for the manual elevator control command.
     */

    /**
     * Constants for joystick processing and smooth driving.
     */
    public static final class DriveConstants {
        /** Joystick deadband threshold (10%). */
        public static final double kJoystickDeadband = 0.1;
        /** Response curve exponent for translation (2.0 = squared). */
        public static final double kTranslationExponent = 2.0;
        /** Response curve exponent for rotation (2.0 = squared). */
        public static final double kRotationExponent = 2.0;
        /** Slew rate limit for translation inputs (units per second). */
        public static final double kTranslationSlewRate = 3.0;
        /** Slew rate limit for rotation input (units per second). */
        public static final double kRotationSlewRate = 3.0;
    }

    /**
     * Constants for the Vision subsystem and dual Limelight configuration.
     */
    // To upload fmap:
    // curl -X POST http://10.90.36.15:5807/upload-fieldmap -H "Content-Type: application/json" --data-binary @FRC2026_ANDYMARK.fmap
    public static final class VisionConstants {
        /** Limelight name for odometry in simulation. */
        public static final String kLimelightNameSim = "limelight-sim";
        public static final String kLimelightNameSim2 = "limelight-sim2";

        // Camera position relative to robot center
        // Example: mounted facing forward, 0.5m forward of center, 0.5m up from center
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)
        );

        // Second camera position relative to robot center
        public static final Transform3d kRobotToCam2 = new Transform3d(
            new Translation3d(-0.5, 0.0, 0.5),
            new Rotation3d(0, 0, Math.PI)
        );

        /** Default value for the VisionEnabled toggle (true = vision on). */
        public static final boolean kVisionEnabledDefault = true;
    }

    /**
     * Constants for the Vision-only Kalman filter used for precise
     * stationary position estimation with multi-tag measurements.
     */
    public static final class VisionKalmanConstants {
        // Initial covariance (high uncertainty at start)
        /** Initial position variance in m². */
        public static final double kInitialPositionVariance = 1.0;
        /** Initial angle variance in rad². */
        public static final double kInitialAngleVariance = 0.5;

        // Process noise Q (very small - robot is stationary)
        /** Position process noise in m². */
        public static final double kProcessNoisePosition = 0.0001;
        /** Angle process noise in rad². */
        public static final double kProcessNoiseAngle = 0.00005;

        // Measurement noise R base values (before tag count scaling)
        /** Base position measurement noise in m². */
        public static final double kBasePositionNoise = 0.05;
        /** Base angle measurement noise in rad². */
        public static final double kBaseAngleNoise = 0.02;

        // Convergence thresholds
        /** Position std dev threshold for convergence (meters). */
        public static final double kConvergencePositionThreshold = 0.02;
        /** Angle std dev threshold for convergence (degrees). */
        public static final double kConvergenceAngleThresholdDegrees = 1.0;

        // Motionless detection
        /** Gyro angular rate threshold for "motionless" detection (deg/sec). */
        public static final double kMotionlessGyroThreshold = 2.0;
        /** Linear velocity threshold for "motionless" detection (m/s). */
        public static final double kMotionlessLinearThreshold = 0.05;
    }
}
