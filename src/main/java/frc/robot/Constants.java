// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.LinearVelocity;

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
        /** Default speed for pulling game pieces into the robot. */
        public static final double kIntakeSpeed = 0.8;
        /** Default speed for ejecting game pieces from the robot. */
        public static final double kOuttakeSpeed = -0.8;
        /** Current threshold in Amps used to detect if a game piece is fully secured. */
        public static final int kIntakeRollerStallLimit = 40;

        public static final int kIntakeMotorID = 22;

        public static final double kIntakeRollerGearRatio = 3.0;
    }

    public static final class ArmConstants {
        /** Default speed for raising and lowering the intake arm. */
        public static final double kArmSpeed = 0.15;
        public static final int kArmStallLimit = 40;

        /** Arm speed during the wiggle command (0.0 to 1.0). */
        public static final double kArmWigglePercent = 0.5;

        public static final int kLeftArmMotorID = 20;
        public static final int kRightArmMotorID = 21;

        /** Max angle (degrees) — lowered/deployed position.
         * Measured on hardware: 2/23/2026
        */
        public static final double kMaxArmAngle = 160;

        /** Min angle (degrees) — raised/stowed position. */
        public static final double kMinArmAngle = 10;

        /** Position loop proportional gain. TODO: tune on hardware — 0.05 is a conservative starting value. */
        public static final double kArmPositionP = 0.05;

        // Arm homing (hard-stop) behavior

        /** Open-loop speed used while homing toward the hard stop.
         * Negative sign means arm homes towards up position.
         * $TODO verify on hardware — too low won't produce detectable stall current */
        public static final double kArmHomingSpeed = -0.05;
        /** Current (Amps) above which we consider the arm "stalled" against a stop while homing.
         * $TODO verify on hardware — must be achievable at kArmHomingSpeed but above free-spin current */
        public static final double kArmHomingStallCurrent = 15.0;
        /** Arm output velocity (degrees per second) below which we consider motion stopped. */
        public static final double kArmHomingStallVelocity = 5.0;
        /** How long the stall condition must be continuously true to count as homed (seconds). */
        public static final double kArmHomingStallSeconds = 0.15;
        /** Overall timeout for the homing routine (seconds). */
        public static final double kArmHomingTimeoutSeconds = 3.5;

        public static final double kArmGearRatio = 20.0;
    }

    public static final class SimIntakeConstants {
        public static final String kDeviceName = "IntakeSim";
        public static final double kMoiKgM2 = 0.001;
    }

    public static final class SimIntakeArmConstants {
        public static final String kDeviceName = "IntakeArmSim";
        public static final double kMoiKgM2 = 0.25;
        public static final double kArmLengthMeters = 0.45;
    }

    public static final class SimShooterConstants {
        public static final String kDeviceName = "ShooterSim";
        public static final double kMoiKgM2 = 0.001;
    }

    public static final class SimIndexerConstants {
        public static final String kDeviceName = "IndexerSim";
        public static final double kMoiKgM2 = 0.001;
    }

    public static final class SimSpinnyWheelsConstants {
        public static final String kDeviceName = "SpinnyWheelsSim";
        public static final double kMoiKgM2 = 0.001;
    }

    public static final class SimClimberConstants {
        public static final String kDeviceName = "ClimberSim";
        /** Motor-to-mechanism gear ratio. */
        public static final double kGearRatio = 20.0;
        /** Mass of carriage + robot being lifted (kg). */
        public static final double kCarriageMassKg = 60.0; // TODO: filler (~130 lb robot)
        /** Radius of the winch drum (m). */
        public static final double kDrumRadiusMeters = 0.02; // TODO: filler
        /** Minimum carriage height (m). */
        public static final double kMinHeightMeters = 0.0;
        /** Maximum carriage height (m). */
        public static final double kMaxHeightMeters = 0.5; // TODO: filler
    }

    /**
     * Constants for shooter.
     */
    public static final class ShooterConstants {
        /** CAN ID for the left SparkFlex motor controller. */
        public static final int kLMotorID = 40;
        /** CAN ID for the right SparkFlex motor controller. */
        public static final int kRMotorID = 41;

        /** SmartCurrentLimit in amps*/
        public static final int kCurrentSupplyLimit = 50; //TODO: filler value
        public static final int kCurrentStatorLimit = 80; // Value tested with CL

        /** PWM channer for the linear actuator servo. */
        public static final int kHoodPwmChannel = 0; //TODO: filler value

        /** Shooting speed */
        public static final double kShootSpeed = 0.60; //TODO: filler value, tested 0.80 with CL and may increase to that later

        public static final double kShooterGearRatio = 1.0;
    }

    /**
     * Constants for Indexer.
     */
    public static final class IndexerConstants {
        public static final int kMotorID = 30; //TODO: filler value
        /** Indexer flywheel speed */
        public static final double kIndexSpeed = 0.90; //TODO: filler value

        public static final int kIndexerCurrentLimit = 50; //TODO: filler value

        public static final double kIndexerGearRatio = 4.0;
        /** The shooter spin-up time before indexer activation, in seconds */
        public static final double kIndexDelay = 0.3;
    }

    /**
     * Constants for the Spinny Wheels (Always-on mechanism).
     */
    public static final class SpinnyWheelsConstants {
        /** CAN ID for the SparkFlex motor. */
        public static final int kMotorID = 51;

        /** Speed for the spinny wheel */
        public static final double kSpinSpeed = 0.05;

        public static final double kSpinGearRatio = 1.0;

        public static final int kCurrentLimit = 40;

        /** Total time of 1 clockwise and 1 counter-clockwise segment */
        public static final Time kTotalTime = Seconds.of(3);
        /** Time it spins clockwise for at the start of each segment */
        public static final Time kClockwiseTime = Seconds.of(2);
    }

    /**
     * Constants for the Climber subsystem (Hook).
     */
    public static final class ClimberConstants {
        /** CAN ID for the single climb motor. */
        public static final int kClimberMotorID = 50;

        public static final double kGearRatio = 20;
        /** Maximum extension limit (motor rotations). */
        public static final double kMaxHeight = 4.0 * kGearRatio;
        /** Minimum retraction limit. (motor rotations) */
        public static final double kMinHeight = 0.1 * kGearRatio;

        /** Amps limit (higher for single motor lifting full weight). */
        public static final int kCurrentLimit = 80;
        public static final double kClimbUpSpeed = 0.4;   // Positive usually extends
        public static final double kClimbDownSpeed = -0.4; // Negative usually retracts
        public static final double kClimbSlewRate = 2.0;
    }

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

        public static final LinearVelocity kJiggleSpeed = MetersPerSecond.of(0.5); //TODO: filler value
        public static final Time kSecondsToAlternate = Seconds.of(0.25); //TODO: filler value
    }

    /** Constants for auto commands. */
    public static final class AutoConstants {
        public static final double k_raiseClimbDuration = 4.0;
        public static final double k_lowerClimbDuration = 4.0;

        public static final double k_getFuelDuration = 3.0;

        public static final double k_intakeBottomDuration = 3.5;
        public static final double k_intakeTopDuration = 2.0;

        public static final double k_shootCommandTotalDuration = 8.0;
    }
}
