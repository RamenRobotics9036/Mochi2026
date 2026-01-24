// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.sim.GroundTruthSimFactory;
import frc.robot.sim.GroundTruthSimInterface;
import frc.robot.sim.SimJoystickOrientation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.auto.AutoLogic;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Swerve drive requests */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.0001)
            .withRotationalDeadband(0.0001)
            .withDriveRequestType(DriveRequestType.Velocity); // $TODO - Why do samples use DriveRequestType.OpenLoopVoltage instead?

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    // Ground truth simulation for testing vision correction
    public GroundTruthSimInterface groundTruthSim = null;

    private Consumer<Pose2d> visionResetter;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /** Stores the starting pose of the currently selected auto */
    private Pose2d selectedAutoStartingPose = new Pose2d();

    public RobotContainer() {
        // Ground truth simulation setup
        if (Robot.isSimulation()) {
            groundTruthSim = GroundTruthSimFactory.create(drivetrain, this::resetRobotPose);
        }

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

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Bumper buttons for simulation
        if (Robot.isSimulation() && groundTruthSim != null) {
            // In simulation, inject drift with right bumper to test vision correction
            joystick.rightBumper().onTrue(drivetrain.runOnce(() ->
                groundTruthSim.injectDrift(0.5, 15.0)  // 0.5m translation, 15° rotation drift
            ));

            // Left bumper resets robot to the starting pose of the selected auto
            joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
                groundTruthSim.cycleResetPosition(selectedAutoStartingPose)
            ));
        }

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

    /**
     * Sets the vision resetter consumer to be called when the robot pose is reset.
     * @param resetter Consumer that accepts a Pose2d to reset vision position
     */
    public void setVisionResetter(Consumer<Pose2d> resetter) {
        this.visionResetter = resetter;
    }

    /**
     * Called when the robot pose is reset in simulation.
     * This is triggered by GroundTruthSim via the consumer pattern.
     *
     * Resets both the ground truth pose and the drivetrain pose to the specified pose.
     * Also resets the vision system simulation pose history if a Vision instance is set.
     *
     * @param pose The new pose the robot has been reset to
     */
    private void resetRobotPose(Pose2d pose) {
        System.out.println("Robot pose reset to: " + pose);

        if (Robot.isSimulation() && groundTruthSim != null) {
            groundTruthSim.resetGroundTruthPoseForSim(pose);
        }

        drivetrain.resetPose(pose);

        if (visionResetter != null) {
            visionResetter.accept(pose);
        }
    }
}
