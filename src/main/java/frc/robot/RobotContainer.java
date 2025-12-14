// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.auto.AutoLogic;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Swerve drive requests */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        // Log swerve configuration on startup
        System.out.println("=== Swerve Configuration ===");
        System.out.println("Drive Motor Stator Current Limit: " + Constants.SwerveConstants.kDriveStatorCurrentLimit + " Amps");
        System.out.println("Steer Motor Stator Current Limit: " + Constants.SwerveConstants.kSteerStatorCurrentLimit + " Amps");
        System.out.println("CAN Bus Name: " + (Constants.SwerveConstants.kCANBusName.isEmpty() ? "RIO (default)" : Constants.SwerveConstants.kCANBusName));
        System.out.println("CAN Bus Log Path: " + Constants.SwerveConstants.kCANBusLogPath);
        
        // Check actual connection status
        var status = TunerConstants.kCANBus.isNetworkFD();
        if (status) {
            System.out.println("CAN Bus Status: Bang Bang it works! (CAN FD Detected)");
        } else {
            System.out.println("CAN Bus Status: STANDARD (CAN 2.0 or RIO Internal) - Check if CAN FD was expected.");
        }
        System.out.println("============================");
        
        configureBindings();
    }

    private void configureBindings() {
        // Default drivetrain command
        // Robot's physical forward is aligned with code's Y-axis (90° rotated from standard)
        // So we swap: joystick forward (getLeftY) → VelocityY, joystick strafe (getLeftX) → VelocityX
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftX() * MaxSpeed)  // Strafe left/right
                     .withVelocityY(-joystick.getLeftY() * MaxSpeed)  // Forward/backward
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)  // Rotation
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

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
     * Returns the autonomous command using your team's AutoLogic system
     */
    public Command getAutonomousCommand() {
        return AutoLogic.getAutoCommand(AutoLogic.autoPicker.getSelected());
    }
}
