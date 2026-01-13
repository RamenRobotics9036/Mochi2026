// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.ElevatorToPositionCommand;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.subsystems.auto.AutoLogic;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based
 * is a "declarative" paradigm, very little robot logic should actually be handled
 * in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger
 * mappings) should be declared here.
 */
public class RobotContainer {

    // Maximum linear speed of the robot (from TunerConstants)
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // Maximum angular rate of the robot
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* ===================== SUBSYSTEMS ===================== */
    // The main drivetrain
    public final CommandSwerveDrivetrain drivetrain =
            TunerConstants.createDrivetrain();

    // Vision subsystem for Limelight MegaTag2 pose estimation
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    // Elevator subsystem (from the original RobotContainer)
    private final ElevatorSystem elevator = new ElevatorSystem();

    /* ===================== CONTROLLERS ===================== */
    // Standard Xbox controller for driver and operator input
    private final CommandXboxController joystick = new CommandXboxController(0);

    /* ===================== SWERVE REQUESTS ===================== */
    // Field-centric drive request for velocity control (newest convention)
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.0001)
            .withRotationalDeadband(0.0001)
            .withDriveRequestType(DriveRequestType.Velocity);

    // Brake request to stop the robot
    private final SwerveRequest.SwerveDriveBrake brake =
            new SwerveRequest.SwerveDriveBrake();

    // Point wheels in a specific direction (used for testing or facing a target)
    private final SwerveRequest.PointWheelsAt point =
            new SwerveRequest.PointWheelsAt();

    // Telemetry logger
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* ===================== CONSTRUCTOR ===================== */
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure controller bindings
        configureBindings();
    }

    // takes the X value from the joystick, and applies a deadband and input scaling
    private double getDriveX() {
        // Joystick +Y is back, Robot +X is forward
        double input = MathUtil.applyDeadband(-joystick.getLeftY(), 0.1);
        // Using Right Bumper as the 'driveSlowMode' toggle
        double inputScale = joystick.rightBumper().getAsBoolean() ? 0.5 : 1.0;
        return input * MaxSpeed * inputScale;
    }

    // takes the Y value from the joystick, and applies a deadband and input scaling
    private double getDriveY() {
        // Joystick +X is right, Robot +Y is left (standard FieldCentric convention)
        double input = MathUtil.applyDeadband(-joystick.getLeftX(), 0.1);
        double inputScale = joystick.rightBumper().getAsBoolean() ? 0.5 : 1.0;
        return input * MaxSpeed * inputScale;
    }

    // takes the rotation value from the joystick, and applies a deadband and input scaling
    private double getDriveRotate() {
        // Joystick +X is right, Robot +angle is CCW (left)
        double input = MathUtil.applyDeadband(-joystick.getRightX(), 0.1);
        double inputScale = joystick.rightBumper().getAsBoolean() ? 0.5 : 1.0;
        return input * MaxAngularRate * inputScale;
    }

    /* ===================== BINDINGS ===================== */
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {

        /* -------- DRIVETRAIN DEFAULT -------- */
        // Default drivetrain command using joystick input
        // Standard WPILib convention: +X is forward, +Y is left
        // Joystick: pushing forward = negative Y, pushing left = negative X
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(getDriveX())
                     .withVelocityY(getDriveY())
                     .withRotationalRate(getDriveRotate())
            )
        );

        /* -------- ELEVATOR DEFAULT -------- */
        elevator.setDefaultCommand(
            new ElevatorDefaultCommand(
                elevator, 
                () -> joystick.getRightY() // Manual control with right stick Y
            )
        );

        /* -------- DISABLED MODE -------- */
        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /* -------- SWERVE BUTTONS -------- */
        // Hold A to brake
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Hold B to point wheels in the direction of joystick left stick
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(
                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX())
            )
        ));

        // SysID commands for drivetrain characterization
        joystick.back().and(joystick.y())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        joystick.back().and(joystick.x())
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        joystick.start().and(joystick.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        joystick.start().and(joystick.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Left bumper seeds field-centric orientation
        joystick.leftBumper()
                .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        /* ===================== ELEVATOR CONTROLS ===================== */
        // These are lifted directly from the original RobotContainer

        // L2 Preset
        joystick.x().onTrue(
            CmdWrapperTeamCommand(
                new ElevatorToPositionCommand(
                    elevator,
                    ElevatorConstants.kLevel2ReefPosition
                )
            )
        );

        // L3 Preset
        joystick.y().onTrue(
            CmdWrapperTeamCommand(
                new ElevatorToPositionCommand(
                    elevator,
                    ElevatorConstants.kLevel3ReefPosition
                )
            )
        );

        // L4 Preset
        joystick.povUp().onTrue(
            CmdWrapperTeamCommand(
                new ElevatorToPositionCommand(
                    elevator,
                    ElevatorConstants.kLevel4ReefPosition
                )
            )
        );

        // Elevator Down
        joystick.povDown().onTrue(
            CmdWrapperTeamCommand(
                new ElevatorToPositionCommand(
                    elevator,
                    ElevatorConstants.kDownElevatorPosition
                )
            )
        );

        /* -------- TELEMETRY -------- */
        // Continuously log drivetrain telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /* ===================== SIM SAFETY ===================== */
    /**
     * Simulation-safe wrapper for team commands (placeholder for future team commands)
     */
    private Command CmdWrapperTeamCommand(Command command) {
        if (RobotBase.isSimulation()) {
            return Commands.none(); // disable commands in simulation if needed
        }
        return command;
    }

    /* ===================== AUTONOMOUS ===================== */
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //This is the Autologic that is used throughout Path Planner
        return AutoLogic.getAutoCommand(AutoLogic.autoPicker.getSelected());
    }
}
