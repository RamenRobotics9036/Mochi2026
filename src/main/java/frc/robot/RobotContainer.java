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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.ElevatorToPositionCommand;
import frc.robot.commands.ElevatorDefaultCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.auto.AutoLogic;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Swerve drive requests */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.0001)
            .withRotationalDeadband(0.0001)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Elevator subsystem
    private final ElevatorSystem elevator = new ElevatorSystem();
    
    // Intake subsystem
    private final IntakeSubsystem intake = new IntakeSubsystem();

    public RobotContainer() {      
        AutoLogic.initShuffleboard(drivetrain); 
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

    private void configureBindings() {
        // Default drivetrain command using custom helper methods
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(getDriveX())
                     .withVelocityY(getDriveY())
                     .withRotationalRate(getDriveRotate())
            )
        );

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Joystick button bindings
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Back Button: Reset Gym Heading (Field Centric)
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* -------- ELEVATOR DEFAULT -------- */
        elevator.setDefaultCommand(
            new ElevatorDefaultCommand(
                elevator, 
                () -> joystick.getRightY() // Manual control with right stick Y
            )
        );

        /* ===================== INTAKE CONTROLS ===================== */
        // Right Trigger: Run Intake until stall
        joystick.rightTrigger()
                .onTrue(new IntakeCommand(intake));

        // Left Trigger: Run Outtake while held
        joystick.leftTrigger()
                .whileTrue(Commands.run(() -> intake.setSpeed(frc.robot.Constants.IntakeConstants.kOuttakeSpeed), intake))
                .onFalse(Commands.runOnce(intake::stop, intake));

        /* ===================== ELEVATOR CONTROLS ===================== */
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

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Simulation-safe wrapper for team commands (placeholder for future team commands)
     */
    private Command CmdWrapperTeamCommand(Command command) {
        if (RobotBase.isSimulation()) {
            return Commands.none(); // disable commands in simulation if needed
        }
        return command;
    }
    
    /**
     * Returns the autonomous command selected on the dashboard.
     */
    public Command getAutonomousCommand() {
        return AutoLogic.getSelectedAutoCommand();
    }
}
