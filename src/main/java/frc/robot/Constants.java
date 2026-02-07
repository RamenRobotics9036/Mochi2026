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
     * Constants for the Elevator subsystem hardware and control limits.
     */
    public static final class ElevatorConstants {
        /** CAN ID for the leader SPARK MAX motor controller. */
        public static final int kLeaderMotorID = 20;
        /** CAN ID for the follower SPARK MAX motor controller. */
        public static final int kFollowMotorID = 21;
        /** Digital Input/Output port for the homing limit switch. */
        public static final int kDIOIndex = 0;

        /** The maximum allowable height for the elevator (Software Soft Limit). */
        public static final double kMaxElevatorPosition = -100.0;
        /** The bottom-most position for the elevator, usually zeroed at the limit switch. */
        public static final double kDownElevatorPosition = 0.0;

        /** Position setpoint for scoring on Reef Level 2. */
        public static final double kLevel2ReefPosition = -25.0;
        /** Position setpoint for scoring on Reef Level 3. */
        public static final double kLevel3ReefPosition = -50.0;
        /** Position setpoint for scoring on Reef Level 4. */
        public static final double kLevel4ReefPosition = -75.0;

        /** Acceptable error range (deadband) for determining if the elevator has reached its target. */
        public static final double tolerance = 2.0;

        /** Factor to convert motor rotations into real-world linear units. */
        public static final double kRotationToElevatorRatio = 1.0;
        /** Maximum percent output for the elevator motors (0.0 to 1.0). */
        public static final double elevatorMaxSpeed = 0.5;
        /** Current limit in Amps to prevent motor damage during a mechanical stall. */
        public static final int kElevatorStallLimit = 40; // Amps
    }

    /**
     * Constants for the 2026 Intake subsystems.
     */
    public static final class IntakeConstants {
        private IntakeConstants() {
            throw new UnsupportedOperationException("This is a utility class!");
        }

        /**
         * Intake lift configuration (dual SPARK FLEX + NEO Vortex).
         * Encoder units are rotations.
         */
        public static final class Lift {
            /** CAN ID for the left lift SPARK FLEX. */
            public static final int kLeftMotorId = 31;
            /** CAN ID for the right lift SPARK FLEX. */
            public static final int kRightMotorId = 32;

            /** True if the left lift motor should be inverted. */
            public static final boolean kLeftMotorInverted = false;
            /** True if the right lift motor should be inverted. */
            public static final boolean kRightMotorInverted = true;

            /** Closed-loop PID gains for position control. */
            public static final double kP = 3.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            /** Feedforward gains (volts) for gravity compensation. */
            public static final double kS = 0.0;
            public static final double kG = 0.35;
            public static final double kV = 0.0;
            public static final double kA = 0.0;

            /**
             * Conversion from encoder rotations to arm angle (radians).
             * This must match the lift's gear ratio and geometry.
             */
            public static final double kRotationsToRadians = 2.0 * Math.PI;
            /** Angle offset in radians applied to the feedforward model. */
            public static final double kAngleOffsetRadians = 0.0;

            /** Target positions for the lift in rotations. */
            public static final double kStowedPositionRotations = 0.0;
            public static final double kDeployedPositionRotations = 12.0;

            /** Soft limit buffer added beyond the min/max target positions. */
            public static final double kSoftLimitBufferRotations = 0.5;
            /** Forward soft limit (rotations). */
            public static final double kSoftLimitForwardRotations =
                Math.max(kStowedPositionRotations, kDeployedPositionRotations) + kSoftLimitBufferRotations;
            /** Reverse soft limit (rotations). */
            public static final double kSoftLimitReverseRotations =
                Math.min(kStowedPositionRotations, kDeployedPositionRotations) - kSoftLimitBufferRotations;

            /** Enable forward/reverse limit switches (normally-closed wiring). */
            public static final boolean kEnableForwardLimitSwitch = true;
            public static final boolean kEnableReverseLimitSwitch = true;

            /**
             * Output range for the lift closed loop (percent output).
             * Keep this conservative to protect the arm and gearbox.
             */
            public static final double kMinOutput = -0.8;
            public static final double kMaxOutput = 0.8;

            /** Smart current limit for each lift motor (amps). */
            public static final int kSmartCurrentLimitAmps = 50;
            /** Current threshold (amps) used for stall detection. */
            public static final double kStallCurrentThresholdAmps = 60.0;
            /** Minimum applied output required to evaluate stall detection. */
            public static final double kStallMinAppliedOutput = 0.2;
            /** Time the current must exceed the threshold before a stall fault is latched. */
            public static final double kStallTimeSeconds = 0.25;

            /** Tolerance for determining if the lift has reached its target (rotations). */
            public static final double kPositionToleranceRotations = 0.15;
        }

        /**
         * Intake roller configuration (single SPARK MAX + NEO Vortex).
         */
        public static final class Roller {
            /** CAN ID for the intake roller SPARK MAX. */
            public static final int kRollerMotorId = 30;

            /** True if positive output should correspond to intake direction (CCW). */
            public static final boolean kMotorInverted = false;

            /** Percent outputs for roller modes (positive = intake direction). */
            public static final double kIntakeSpeed = 0.75;
            public static final double kOuttakeSpeed = 0.75;
            public static final double kEjectJamSpeed = 0.6;

            /** Smart current limit for the roller motor (amps). */
            public static final int kSmartCurrentLimitAmps = 40;

            /** Minimum applied output required before current-based detection is evaluated. */
            public static final double kMinOutputForDetection = 0.1;

            /** Current threshold (amps) used to detect a secured game piece. */
            public static final double kPieceDetectCurrentThresholdAmps = 35.0;
            /** Debounce time for piece detection (seconds). */
            public static final double kPieceDetectTimeSeconds = 0.15;

            /** Current threshold (amps) used for jam detection. */
            public static final double kJamCurrentThresholdAmps = 50.0;
            /** Debounce time for jam detection (seconds). */
            public static final double kJamDetectTimeSeconds = 0.10;
        }

        /**
         * Intake sequence timing and safety limits.
         */
        public static final class Sequence {
            /** Max time allowed to deploy the lift before faulting (seconds). */
            public static final double kDeployTimeoutSeconds = 1.5;
            /** Max time allowed to stow the lift before faulting (seconds). */
            public static final double kStowTimeoutSeconds = 1.5;
            /** Time to reverse the roller when a jam is detected (seconds). */
            public static final double kJamReverseSeconds = 0.25;
        }
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
    public static final class ElevatorDefaultCommandConstants {
        /** Sensitivity multiplier for joystick-based elevator movement. */
        public static final double kElevatorSpeed = 0.5;
    }

    /**
     * Constants for the Vision subsystem and dual Limelight configuration.
     */
    public static final class VisionConstants {
        /** Network table name for the chassis-mounted localization camera. */
        public static final String kFixedCameraName = "limelight-fixed";

        /** Network table name for the turret-mounted tracking camera. */
        public static final String kTurretCameraName = "limelight-turret";

        /** Index of the Limelight pipeline configured for AprilTag 3D localization. */
        public static final int PIPELINE_TAGS = 0;

        /** Index of the Limelight pipeline configured for color-based object detection. */
        public static final int PIPELINE_FUEL = 1;

        /** Vertical angle threshold to ignore game pieces that are too far away. */
        public static final double FUEL_TY_FILTER = -15.0;

        /** The physical 3D offset of the fixed camera relative to the robot's center. */
        public static final Transform3d kFixedCameraTransform = new Transform3d(
            new Translation3d(
                0.25,   // 25cm forward from robot center
                0.0,    // Centered left-right
                0.50    // 50cm up from ground
            ),
            new Rotation3d(
                0.0,                            // Roll: 0
                Math.toRadians(-15.0),          // Pitch: 15° down
                0.0                             // Yaw: 0 (facing forward)
            )
        );

        /** Distance threshold in meters beyond which AprilTag data is considered too noisy to trust. */
        public static final double MAX_TAG_DISTANCE = 4.0;

        /** Maximum distance allowed to execute a manual 'Snap-to-Pose' realignment. */
        public static final double SNAP_MAX_DISTANCE = 3.5;

        /** Number of visible tags required before the Pose Estimator accepts a vision update. */
        public static final int MIN_TAG_COUNT = 1;

        /** Default camera name for legacy support modules. */
        public static final String LIMELIGHT_NAME = kFixedCameraName;
        /** The 0,0,0 coordinate reference for the 2026 FRC field. */
        public static final Pose2d FIELD_ORIGIN = new Pose2d();
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
