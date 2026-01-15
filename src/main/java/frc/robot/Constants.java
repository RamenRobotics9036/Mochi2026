// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

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
        public static final int kElevatorStallLimit = 40; // Amps
    }

    /**
     * Constants for the Intake subsystem.
     */
    public static final class IntakeConstants {
        public static final int kIntakeMotorID = 30;
        public static final double kIntakeSpeed = 0.8;
        public static final double kOuttakeSpeed = -0.8;
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
}
