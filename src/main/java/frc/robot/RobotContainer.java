// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final double MaxSpeed = 2; // actual applied max speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // Max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15)
            .withRotationalDeadband(MaxAngularRate * 0.15) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Request used for both Static Aiming and Shoot on the Move (Robot-Centric tracking)
    private final SwerveRequest.RobotCentric robotCentricAimRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Default teleop driving command
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(joystick.getLeftY() * MaxSpeed)
                .withVelocityY(joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );  

        // Hold 'A' Button: Shoot on the Move
        // Manual stick strafe, automatic target heading and distance tracking
        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> robotCentricAimRequest
                .withVelocityX(getLimelightForwardSpeed())     // Auto-distance
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Manual strafe
                .withRotationalRate(getLimelightRotationSpeed()) // Auto-heading
            )
        );

        // Hold 'Y' Button: Static Aim and Range
        // Complete hands-off control. The robot automatically drives to the target and aligns
        joystick.y().whileTrue(
            drivetrain.applyRequest(() -> robotCentricAimRequest
                .withVelocityX(getLimelightForwardSpeed())
                .withVelocityY(0.0) 
                .withRotationalRate(getLimelightRotationSpeed())
            )
        );

        // Hold 'B' Button: Lock modules to cross-brake configuration
        joystick.b().whileTrue(drivetrain.applyRequest(() -> brake));

        // Hold 'X' Button: Point wheels at joystick vector direction
        joystick.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Idle while the robot is disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // Used by 'A' and 'Y' Buttons to control rotation
    private double getLimelightRotationSpeed() {
        if (!LimelightHelpers.getTV("limelight")) {
            return 0.0;
        }

        double tx = LimelightHelpers.getTX("limelight");
        if (Math.abs(tx) < 1.5) {
            return 0.0;
        }

        double kP = 0.015; 
        
        // Note: If the robot still spins rapidly when tracking, 
        // change "-MaxAngularRate" to "MaxAngularRate" to invert the direction.
        return tx * kP * -MaxAngularRate;
    }
    
    // Used by 'A' and 'Y' Buttons to control forward/backward distance
    private double getLimelightForwardSpeed() {
        if (!LimelightHelpers.getTV("limelight")) {
            return 0.0;
        }

        double kP = 0.04;
        double desiredTY = 0.0; 
        double currentTY = LimelightHelpers.getTY("limelight");
        double error = currentTY - desiredTY;
    
        if (Math.abs(error) < 1.0) {
            return 0.0;
        }
    
        double targetingForwardSpeed = error * kP * MaxSpeed * -1.0;
        return edu.wpi.first.math.MathUtil.clamp(targetingForwardSpeed, -MaxSpeed * 0.4, MaxSpeed * 0.4);
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }
}