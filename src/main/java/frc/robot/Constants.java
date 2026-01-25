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
}
