// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.RobotIdentity;
import frc.robot.commands.FullAutoClimbCommand;
import frc.robot.commands.GetFuelCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.IntakeArmHomeCommand;
import frc.robot.commands.SetIntakeBottomCommand;
import frc.robot.commands.SetIntakeTopCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JiggleCommand;
import frc.robot.commands.RotateToTargetCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.SpinnyDefaultCommand;
import frc.robot.sim.JoystickInputsRecord;
import frc.robot.sim.RollerSim.RollerIoInterface;
import frc.robot.sim.RollerSim.TwoMotorRollerIoInterface;
import frc.robot.sim.SimIoFactory;
import frc.robot.sim.ShowVisionOnField;
import frc.robot.sim.elevatorSim.ElevatorIoInterface;
import frc.robot.subsystems.climber.ClimberIoReal;
import frc.robot.sim.SimWrapper;
import frc.robot.sim.armsim.ArmIoInterface;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpinnyWheels;
import frc.robot.subsystems.TestSubsystems;
import frc.robot.auto.AutoLogic;
import frc.robot.subsystems.indexer.IndexerIoReal;
import frc.robot.subsystems.intake.ArmIoReal;
import frc.robot.subsystems.intake.IntakeIoReal;
import frc.robot.subsystems.shooter.ShooterIoReal;
import frc.robot.visutils.AimController;
import frc.robot.visutils.BasicInfoDashboard;
import frc.robot.visutils.DashboardFactory;
import frc.robot.visutils.DriveAccuracyTester;
import frc.robot.visutils.DriveSmooth;
import frc.robot.visutils.MotionlessTracker;
import frc.robot.visutils.MultiCamOdometry;
import frc.robot.visutils.MultiCamOdometryFactory;
import frc.robot.visutils.PerCycleState.CameraSelectionMode;
import frc.robot.visutils.TurnToAngleHelper;
import frc.robot.visutils.VisionKalmanFilter;
import java.util.OptionalDouble;


/**
 * The RobotContainer class is where the bulk of the robot structure is declared.
 *
 * <p>This class centralizes the hardware instances, UI device bindings, and
 * the mapping between operator inputs and subsystem commands for the 2026 season.
 */
public class RobotContainer {

    private BotConfigInterface m_configInterface = RobotIdentity.getBotConfig();

    /** Maximum linear velocity of the robot in meters per second. */
    private double MaxSpeed = m_configInterface.getSpeedAt12Volts().in(MetersPerSecond);
    /** Maximum angular velocity of the robot in radians per second. */
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /** Maximum linear velocity of the robot DURING TELEOP in meters per second. */
    private double TeleoperatedSpeed = Math.min(m_configInterface.getSpeedInTeleop().in(MetersPerSecond), MaxSpeed);

    /** Standard field-centric swerve request. Uses Velocity control for smoother movement.
     * Near-zero deadband catches floating-point residuals only;
     * main joystick deadband and smoothing is handled by DriveSmooth. */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.001 * TeleoperatedSpeed)
            .withRotationalDeadband(0.001 * MaxAngularRate)
            .withDriveRequestType(DriveRequestType.Velocity);

    /** Stateful aim-while-driving controller (PID + edge detection). */
    private final AimController m_aimController =
        new AimController(MaxAngularRate);

    /** Brake request: Forces all modules into an X-pattern to resist movement. */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    /** Point request: Orients all modules toward a specific direction without driving. */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /** Telemetry helper for logging drivetrain state to AdvantageScope/Dashboard. */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /** Primary driver controller (Port 0). */
    private final CommandXboxController driveController = new CommandXboxController(0);
    /** Primary operator controller (Port 1). */
    private final CommandXboxController operateController = new CommandXboxController(1);

    /**
     * Joystick processing pipeline: deadband + response curve + slew-rate limiting.
     * Owned here (not inside JoystickInput) so we can reset it on mode transitions
     * (e.g. auto → teleop).
     */
    private final DriveSmooth m_driveSmooth = new DriveSmooth();

    /** Processes raw joystick inputs into scaled robot velocities. */
    public final JoystickInput m_joystickInput;

    /** The drivetrain subsystem, initialized using constants generated by Tuner X. */
    public final CommandSwerveDrivetrain drivetrain = m_configInterface.createDrivetrain();

    public final BasicInfoDashboard basicInfoDashboard = new BasicInfoDashboard(drivetrain);

    /** Field2d for Glass/SmartDashboard visualization. */
    public final Field2d m_glassField = new Field2d();

    /** Simulation wrapper - null when not in simulation. */
    public final SimWrapper m_simWrapper;
    public final ShowVisionOnField m_showVisionOnField;

    /** Manages the tape-drop accuracy test workflow. */
    public final DriveAccuracyTester m_driveAccuracyTester;

    public final MultiCamOdometry m_multiCamlimelight;

    private final TwoMotorRollerIoInterface m_shooterIO = Robot.isSimulation()
        ? SimIoFactory.createShooterIoSim()
        : new ShooterIoReal(m_configInterface);

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(
        m_configInterface,
        m_shooterIO);

    private final RollerIoInterface m_indexerIO = Robot.isSimulation()
        ? SimIoFactory.createIndexerIoSim()
        : new IndexerIoReal();

    private final RollerIoInterface m_spinnyIO = Robot.isSimulation()
        ? SimIoFactory.createSpinnyIoSim()
        : new frc.robot.subsystems.spinny.SpinnyIoReal();

    public final SpinnyWheels m_spinnyWheels = new SpinnyWheels(m_spinnyIO);

    public final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(m_indexerIO);

    private final ElevatorIoInterface m_climberIO = Robot.isSimulation()
        ? SimIoFactory.createClimberIoSim()
        : new ClimberIoReal();

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem(m_climberIO);

    /** Intake IO: real hardware or FlywheelSim depending on mode. */
    private final RollerIoInterface m_intakeIO = Robot.isSimulation()
        ? SimIoFactory.createIntakeIoSim()
        : new IntakeIoReal();

    private final ArmIoInterface m_intakeArmIO = Robot.isSimulation()
        ? SimIoFactory.createIntakeArmIoSim()
        : new ArmIoReal();

    /** Intake subsystem driven through the IO abstraction. */
    public final IntakeSubsystem intakeSubsystem =
        new IntakeSubsystem(m_intakeIO);

    /** Arm subsystem driven through the IO abstraction. */
    public final ArmSubsystem armSubsystem = new ArmSubsystem(m_intakeArmIO);

    /** Vision-only Kalman filter for precise stationary position estimation. */
    public final VisionKalmanFilter m_visionKalmanFilter = new VisionKalmanFilter();

    /** Tracks whether the robot is motionless and for how long. */
    public final MotionlessTracker m_motionlessTracker;

    /**
     * Constructs the RobotContainer.
     * Initializes autonomous selection dashboards and binds controller inputs to commands.
     */
    public RobotContainer() {
        m_joystickInput = new JoystickInput(
            m_driveSmooth,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            driveController.rightBumper()::getAsBoolean,
            TeleoperatedSpeed,
            MaxAngularRate,
            Robot.isSimulation(),
            () -> drivetrain.getOperatorForwardDirection().getDegrees());

        m_driveAccuracyTester = new DriveAccuracyTester(
            drivetrain, m_visionKalmanFilter, basicInfoDashboard::forceDisableVision);

        m_simWrapper = SimWrapper.create(m_configInterface, drivetrain, this::resetRobotPose);
        m_showVisionOnField = new ShowVisionOnField(
            m_configInterface,
            m_glassField,
            (m_simWrapper == null) ? null : m_simWrapper.getSimDebugField());

        AutoLogic.initShuffleboard(drivetrain);
        DashboardFactory.initDebugDashboard(
            m_configInterface,
            m_glassField,
            m_driveAccuracyTester,
            intakeSubsystem,
            m_indexerSubsystem,
            shooterSubsystem,
            armSubsystem,
            climberSubsystem);

        configureDriveBindings();
        configureOperateBindings();
        configureDefaultCommands();
        registerNamedCommands();

        /** Registers event triggers for PathPlanner */
        new EventTrigger("shoot").onTrue(ShootCommand.create(shooterSubsystem, m_indexerSubsystem));
        new EventTrigger("Full Auto Climb").onTrue(FullAutoClimbCommand.create(climberSubsystem));
        new EventTrigger("get fuel").onTrue(GetFuelCommand.create(intakeSubsystem));
        new EventTrigger("set intake bottom").onTrue(SetIntakeBottomCommand.create(armSubsystem, intakeSubsystem));
        new EventTrigger("set intake top").onTrue(SetIntakeTopCommand.create(armSubsystem, intakeSubsystem));

        // Setup vision system
        m_motionlessTracker = MotionlessTracker.create(
            () -> drivetrain.getState().Speeds,
            m_visionKalmanFilter::reset);

        m_multiCamlimelight = MultiCamOdometryFactory.create(
            m_configInterface,
            drivetrain::addVisionMeasurement,
            basicInfoDashboard,
            m_visionKalmanFilter,
            m_motionlessTracker);
    }

    /** Registers named commands for PathPlanner */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("shoot", ShootCommand.create(shooterSubsystem, m_indexerSubsystem));

        NamedCommands.registerCommand("Full Auto Climb", FullAutoClimbCommand.create(climberSubsystem));

        NamedCommands.registerCommand("get fuel", GetFuelCommand.create(intakeSubsystem));

        NamedCommands.registerCommand("set intake bottom", SetIntakeBottomCommand.create(armSubsystem, intakeSubsystem));

        NamedCommands.registerCommand("set intake top", SetIntakeTopCommand.create(armSubsystem, intakeSubsystem));
    }

    /**
     * Defines trigger-to-command mappings for the driver controller.
     */
    private void configureDriveBindings() {

        // Keep the drivetrain in an Idle state while the robot is disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Map Button A to the brake command for defensive positioning
        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Map Button B to orient wheels based on the left joystick angle (useful for testing)
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        // SysId Characterization bindings (Back/Start + X/Y) for automated PID tuning
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Recalibrate the gyro's forward heading using the Left Bumper
        driveController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Try to align to an AprilTag with the Left Trigger
        driveController.leftTrigger().onTrue(drivetrain.AlignToTag(
            m_configInterface,
            driveController,
            MaxAngularRate));

        // POV buttons for sim
        SimWrapper.configureSimBindings(
            m_simWrapper,
            driveController.povRight(),
            driveController.povLeft(),
            drivetrain);

        // Hook up the telemetry logger to the drivetrain periodic updates
        drivetrain.registerTelemetry(logger::telemeterize);

        // POV Down: rotate in place to face the configured AprilTag.
        // Use a tolerant trigger (135°–225°) instead of exact povDown() (180° only)
        // to avoid command cancellation from D-pad diagonal flicker.
        new Trigger(() -> JoystickInput.isPovDownward(driveController)).whileTrue(
            new RotateToTargetCommand(drivetrain, () ->
                TurnToAngleHelper.getTag2dPose(m_multiCamlimelight.getLastTargetForSingleCam(CameraSelectionMode.CAMERA_BEST_WITH_LOCK))));

        //driveController.y().whileTrue(new JiggleCommand(drivetrain, armSubsystem));
        driveController.y().whileTrue(new JiggleCommand(drivetrain));
    }

    /**
     * Defines trigger-to-command mappings for the operator controller.
     */
    private void configureOperateBindings(){

        // POV Up: Extend Climber
        operateController.povUp().whileTrue(
            new RunCommand(
                () -> climberSubsystem.setClimbSpeed(ClimberConstants.kClimbUpSpeed),
                climberSubsystem
            )
        ).onFalse(new InstantCommand(climberSubsystem::stop, climberSubsystem));

        // POV Down: Retract Climber
        operateController.povDown().whileTrue(
            new RunCommand(
                () -> climberSubsystem.setClimbSpeed(ClimberConstants.kClimbDownSpeed),
                climberSubsystem
            )
        ).onFalse(new InstantCommand(climberSubsystem::stop, climberSubsystem));


        operateController.a().toggleOnTrue(new IntakeCommand(intakeSubsystem));

        // Right stick click: home the arm (must be done before any arm position commands)
        operateController.rightStick().onTrue(
            IntakeArmHomeCommand.create(armSubsystem, intakeSubsystem));

        // Left stick click: set current arm position as the new "zero" reference point
        operateController.leftStick().onTrue(
            Commands.runOnce(armSubsystem::setArmZero, armSubsystem));
    }

    /**
     * Defines default commands.
     */
    private void configureDefaultCommands(){
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                JoystickInputsRecord inputs = m_joystickInput.getJoystickInputs();

                // POV-UP held: auto-aim rotation at the tag
                // while keeping X/Y translation.
                var driveState = drivetrain.getState();
                OptionalDouble aimRate = m_aimController.update(
                    JoystickInput.isPovUpward(driveController),
                    // $TODO - Not sure that CAMERA_BEST is the right param
                    m_multiCamlimelight.getLastTargetForSingleCam(CameraSelectionMode.CAMERA_BEST_WITH_LOCK),
                    driveState.Pose,
                    driveState.Speeds);

                if (aimRate.isPresent()) {
                    return drive
                        .withVelocityX(inputs.driveX())
                        .withVelocityY(inputs.driveY())
                        .withRotationalRate(aimRate.getAsDouble());
                }

                return drive.withVelocityX(inputs.driveX())
                     .withVelocityY(inputs.driveY())
                     .withRotationalRate(inputs.rotatetX());
            })
        );

        //shooterSubsystem.setDefaultCommand(new ShooterTestCommand(shooterSubsystem, operateController));
        shooterSubsystem.setDefaultCommand(new ShooterDefaultCommand(shooterSubsystem, m_indexerSubsystem, operateController));

        m_spinnyWheels.setDefaultCommand(new SpinnyDefaultCommand(m_spinnyWheels));

        armSubsystem.setDefaultCommand(new IntakeArmCommand(armSubsystem, operateController));
    }

    /**
     * Retrieves the autonomous routine selected via the dashboard at the start of the match.
     *
     * @return The command to execute during the autonomous period.
     */
    public Command getAutonomousCommand() {
        return AutoLogic.getSelectedAutoCommand();
    }

    /**
     * Called when the robot pose is reset in simulation.
     * This is triggered by GroundTruthSim via the consumer pattern.
     *
     * <p>Resets both the ground truth pose and the drivetrain pose to the specified pose.
     * Also resets the vision system simulation pose history if a Vision instance is set.
     *
     * @param pose The new pose the robot has been reset to
     */
    private void resetRobotPose(Pose2d pose) {
        System.out.println("Robot pose reset to: " + pose);

        drivetrain.resetPose(pose);

        // $VISIONSIM - Clean reset
        if (Robot.isSimulation()) {
            // NOTE: This is only reset in simulation since m_simWrapper isnt ever
            // instantiated in real mode.  This is because the pose we reset
            // here is the ground truth pose which is only relevent in simulation.
            m_simWrapper.resetSimPose(pose);
        }

        // Reset the vision-only Kalman filter
        m_visionKalmanFilter.reset();

        // Remove any tape from the field
        m_driveAccuracyTester.clearTape();
    }

}
