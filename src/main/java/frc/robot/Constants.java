// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public final class Constants
{
  /**
   * Constants for the joystick.
   */
  public static class OperatorConstants
  {
    public static final int kDriverPort = 0;
    public static final int kArmPort = 1;
    public static final double kExpo = 4; //do not change this value, ever!
    public static final double kExpoRatio = 0.75; // change this 0..1 to add more exponential, 0 = no expo (linear)
    public static final double kDeadband = 0.04;
    public static final Supplier<Alliance> kAlliance = () -> (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() : Alliance.Red;
    public static final boolean kCompetitionMode = false; // Set to true the day of competition to turn off extras (max performance)
    public static final boolean kAllowDetailedPowerLogging = false; // Only when set to true will we log detailed power distrbution logging
    public static final double kRumbleTime = 0.5; // seconds
  }
  /**
   * 
   * Constants for the swerve system.
   */
  public static class SwerveConstants
  {
    // USe the directory matching the robot
    // public static final String  kJsonDirectory = "pancake";
    public static final String  kJsonDirectory = "nori";
    public static final double kMaxSpeedMetersPerSecond = 5.06;
    public static final double kRobotMass = Units.lbsToKilograms(129); // $TODO update
    public static final double kLoopTime = 0.13; //s, 20ms + 110ms sprk max velocity lag
  }
  /**
   * Constants for autonomous mode.
   */
  public static final class AutoConstants
  {
    public static final PIDConstants kTranslationpid = new PIDConstants(0.7, 0, 0); // try 0.7,0,0 from YAGSL-Example
    public static final PIDConstants kAnglePID = new PIDConstants(0.12, 0, 0.00015); // try 0.4,0,0.01 from YAGSL-Example
    public static final double kMaxSpeedMetersPerSecond = 5.06; // try 4.5 from YAGSL-Example
  }

  public static final class VisionConstants {
    public static final String limelightName = "limelight";
    public static final double allowedAngleUncertaintyDegrees = 0.5;
    public static final double allowedAngleUncertaintyMetersDrive = 0.05;
    public static final double allowedAngleUncertaintyMetersStrafe = 0.025;

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }

  public static final class CommandConstants {
    public static final class AlignRobotConstants {
      public static final double maxTimeSeconds = 10;
      public static final double maxSpeed = 0.4;
      public static final double maxSpeedRot = 0.2;

      /**
       * How much the robot should be offset from the April tag pose rotation.
      */
      public static final double transformRot = 0.0;

      /**
       * How much the robot should be offset from the April tag pose x direction.
      */
      public static final double transformDrive = 0.0; // Was 0.5

      public static final double coralOffset = (13.25 / 2) * 0.0254;
      public static final double outTakeOffset = (3.25) * 0.0254;

      /**
       * How much the robot should be offset from the April tag pose y direction.
      */
      public static final double transformStrafe = 3.25 / 12; // 3.25 inches offset
      public static final double transformLeftStrafe = -0.22 - 0.15; // Was coralOffset - outTakeOffset, -18
      public static final double transformRightStrafe = -0.22 + 0.15; // Was -coralOffset - outTakeOffset, -72
    }

    public static final class AimAtLimeLightV2Constants {
      // 5 seconds should be plenty of time to just turn
      public static final double maxTimeSeconds = 5;
      public static final double maxSpeed = 0.4;
      public static final double kMaxRotateRadsPerSecond = 2 * Math.PI;

      public static final double kP = 0.01;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double allowedAngleUncertaintyDegrees = 0.5;
      public static final int dontRotateIfSmallDegrees = 5;

      /**
       * How much the robot should be offset from the April tag pose rotation.
      */
      public static final double transformRot = 0.0;
    }
  }

  public static final class IntakeConstants{
    public static final int kPullMotorID = 21;
    public static final int kLoadMotorID = 20;
    public static final int kStallLimit = 20;
    public static final double kMaxOutputPercentage = 1.0;
    public static final int pullMotorGearBoxFactor= 4;
    public static final int loadMotorGearBoxFactor = 4;
    public static final int kProximitySensorPort = 2;
  }

  public static final class IntakeDefaultCommandConstants {
    public static final double speed = 0.5;
    public static final double pullSpeed = 0.5;
  } 

  public static final class IntakeSpitCommandConstants {
    public static final int maxTime = 3;
    public static final double speed = 0.8;
    public static final double numRotations = 100;
    public static final double bucketSpeed = 0.5;
  }

  public static final class ArmConstants {
    public static final int kArmMotorID = 22;
    public static final double maxOutput = 1.0;
    public static final int kArmEncoderID = 6;
    //public static final double kArmGearBoxRatio = 125 * (44/30);
    //125 is 3-stage gearbox, 38 and 18 are the numbers of teeth on the two gears.
    public static final double kArmGearBoxRatio = 125 * (38/18);
    
    // Make sure the abolute encoder does NOT cross zero as the arm moves from top to bottom
    // This is the position of the arm in the up position, it should ALMOST be touching the endstop
    public static final double kMinArmRotation = 0.49;
    // This is the postition of the arm against the ground
    public static final double kMaxArmRotation = 4.73;
    public static final double L1ArmAngle = 1.11;
    
    public static final double kAbsoluteEncoderOffset = 0;

    public static final int kcurrentLimit = 20;
    public static final double tolerance = 0.1;
    public static final double setArmMaxTime = 4;
    public static final double algaePreset = 1.849222;
    public static final double kP = 1.5;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static final class ArmDefaultCommandConstants {
    public static final double armAngleChangeRate = 16;
  }
  public static final class ElevatorConstants {
  public static final int kLeaderMotorID = 30; 
  public static final int kFollowMotorID = 31; 
  public static final int kDIOIndex = 1; // $TODO placeholder 
  public static final int kStallLimit = 20; 
  public static final double kMaxOutputPercentage = 1; 
  //Elevator moves 5.625 in (0.1429 m) per rotation of the sprocket, gear ratio of 12:1 
  public static final double kRotationToElevatorRatio = (5.625 * 0.0254) / 20; 
  //Physical limit is 43.75 in (1.1112 m) 
  public static final double kMarginOfError = 0.06; 
  public static final double elevatorMaxSpeed = 5; 

  public static final double kDownElevatorPosition = 0.0; 
  public static final double kMaxElevatorPosition = -(0.533+0.1); 
  public static final double kLevel2ReefPosition = -(0.239); 
  public static final double kLevel3ReefPosition = -(0.336 + 0.024); //plus value is to compensate for bent threaded rod. will be replaced saturday 3/29 and should be unnecessary after
  public static final double kLevel4ReefPosition = -(0.533+0.05);
  public static final double maxTime = 2; 
  public static final double tolerance = 0.01; 
  }

  public static final class ElevatorDefaultCommandConstants{
    // In centimeters
    public static final double kElevatorSpeed = 0.5; // $TODO placeholder
  }
  
  public static final class OuttakeConstants {
    public static final int sparkflexID = 32; // No longer used
    public static final int sparkmaxID = 33;
    public static final int currentLimit = 20;
    public static final double motorGearRatio = 1;
  }

  public static final class OuttakeSpitCommandConstants {
    public static final int maxTime = 3;
    public static final double speed = 1.0;
    public static final double numRotations = 250;
  }

  public static final class AutoNameConstants {
    public static final String kCenterL1AutoName = "CENTER 1 Coral L1";
    public static final String kCenterL4AutoName = "CENTER 1 Coral L4";
    public static final String kSideL1AutoName = "SIDE 1 Coral L4";
  }
}
