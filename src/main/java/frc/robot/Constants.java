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
     * Constants for the Elevator subsystem.
     */
    public static final class ElevatorConstants {
        public static final int kLeaderMotorID = 20;
        public static final int kFollowMotorID = 21;
        public static final int kDIOIndex = 0;

        // Elevator position limits (in encoder units)
        public static final double kMaxElevatorPosition = -100.0;
        public static final double kDownElevatorPosition = 0.0;

        // Preset positions for reef levels
        public static final double kLevel2ReefPosition = -25.0;
        public static final double kLevel3ReefPosition = -50.0;
        public static final double kLevel4ReefPosition = -75.0;

        // Position tolerance for commands
        public static final double tolerance = 2.0;

        // Motor configuration
        public static final double kRotationToElevatorRatio = 1.0;
        public static final double elevatorMaxSpeed = 0.5;
    }

    /**
     * Constants for the Intake subsystem.
     */
    public static final class IntakeConstants {
        public static final int kStallLimit = 40; // Amps
    }

    /**
     * Constants for operator interface and dashboard.
     */
    public static final class OperatorConstants {
        public static final boolean kCompetitionMode = false;
    }

    /**
     * Constants for the Elevator default command.
     */
    public static final class ElevatorDefaultCommandConstants {
        public static final double kElevatorSpeed = 0.5;
    }

    /**
     * Constants for the Vision subsystem (Dual Limelight Setup).
     */
    public static final class VisionConstants {
        // ========== CAMERA NAMES (Must match Limelight Web Interface) ==========
        /** Fixed camera on chassis - used for MegaTag 2 localization */
        public static final String kFixedCameraName = "limelight-fixed";
        
        /** Turret-mounted camera - used for target tracking (tx/ty) */
        public static final String kTurretCameraName = "limelight-turret";

        // ========== PIPELINE INDEXES (Must match Limelight Web UI) ==========
        /** Pipeline 0: AprilTag detection for pose estimation */
        public static final int PIPELINE_TAGS = 0;
        
        /** Pipeline 1: Color/GRIP pipeline for fuel (ball) detection */
        public static final int PIPELINE_FUEL = 1;

        // ========== FUEL DETECTION SETTINGS ==========
        /** 
         * Ignore fuel targets above this TY value (too high/far away).
         * Negative TY means target is below crosshair.
         */
        public static final double FUEL_TY_FILTER = -15.0;

        // ========== FIXED CAMERA MOUNT POSITION ==========
        /**
         * Transform from robot center to fixed camera position.
         * Adjust these values based on your actual camera mounting location.
         * 
         * Translation: (forward, left, up) in meters from robot center
         * Rotation: (roll, pitch, yaw) in radians
         */
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

        // ========== POSE ESTIMATION FILTERING ==========
        /** Maximum distance to trust vision estimates (meters) */
        public static final double MAX_TAG_DISTANCE = 4.0;
        
        /** Maximum distance for snap-to-pose command (meters) */
        public static final double SNAP_MAX_DISTANCE = 3.5;
        
        /** Minimum number of tags required for pose update */
        public static final int MIN_TAG_COUNT = 1;

        // Legacy field (kept for compatibility)
        public static final String LIMELIGHT_NAME = kFixedCameraName;
        public static final Pose2d FIELD_ORIGIN = new Pose2d();
    }
}
