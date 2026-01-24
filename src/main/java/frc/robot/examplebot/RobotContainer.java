// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
// $TODO - Ground truth
import frc.robot.sim.GroundTruthSimFactory;
import frc.robot.sim.GroundTruthSimInterface;
// $TODO - Joystick screen orientation
import frc.robot.sim.SimJoystickOrientation;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // $TODO - Ground truth
    public GroundTruthSimInterface groundTruthSim = null;

    private Consumer<Pose2d> visionResetter;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /** Stores the starting pose of the currently selected auto */
    private Pose2d selectedAutoStartingPose = new Pose2d();

    public RobotContainer() {
        // $TODO - Ground truth
        if (Robot.isSimulation()) {
            groundTruthSim = GroundTruthSimFactory.create(drivetrain, this::resetRobotPose);
        }

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        initAutoSelector();

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private Command getJoystickCommandForPhysicalRobot() {
        return drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    // $TODO - Joystick screen orientation
    private Command getJoystickCommandForSimRobot() {
        return drivetrain.applyRequest(() ->
            SimJoystickOrientation.applySimJoystickInput(
                drive,
                drivetrain.getOperatorForwardDirection().getDegrees(),
                joystick,
                MaxSpeed,
                MaxAngularRate));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            Robot.isSimulation()
                // $TODO - Joystick screen orientation
                ? getJoystickCommandForSimRobot()
                : getJoystickCommandForPhysicalRobot()
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // $TODO - Bumper buttons
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

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    /**
     * Initializes the auto selector and publishes it to SmartDashboard.
     */
    private void initAutoSelector() {
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Update starting pose when auto selection changes
        autoChooser.onChange(this::onNewAutoSelected);

        // Initialize starting pose from default selection
        onNewAutoSelected(autoChooser.getSelected());
    }

    /**
     * Called when a new auto is selected from the chooser.
     * Updates the starting pose for simulation reset.
     */
    private void onNewAutoSelected(Command command) {
        if (command instanceof PathPlannerAuto auto) {
            Pose2d pose = auto.getStartingPose();
            selectedAutoStartingPose = pose != null ? pose : new Pose2d();
        } else {
            // "None" selection is an InstantCommand - reset to origin
            selectedAutoStartingPose = new Pose2d();
        }
    }

    /**
     * Sets the vision resetter consumer to be called when the robot pose is reset.
     * @param resetter Consumer that accepts a Pose2d to reset vision position
     */
    // $TODO - Clean reset
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
    // $TODO - Clean reset
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
