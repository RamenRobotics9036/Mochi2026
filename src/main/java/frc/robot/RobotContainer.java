// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.RobotConfigSelector;
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
import frc.robot.sim.ShowVisionOnField;
import frc.robot.sim.SimIoFactory;
import frc.robot.subsystems.climber.ClimberIoReal;
import frc.robot.sim.SimWrapper;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpinnyWheels;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.indexer.IndexerIoReal;
import frc.robot.subsystems.intake.ArmIoReal;
import frc.robot.subsystems.intake.IntakeIoReal;
import frc.robot.subsystems.shooter.ShooterIoReal;
import frc.robot.util.JoyHelpers;
import frc.robot.visutils.AimController;
import frc.robot.visutils.BasicInfoDashboard;
import frc.robot.visutils.CamInputs;
import frc.robot.visutils.CamOdometryInterface;
import frc.robot.visutils.CamOutputs;
import frc.robot.visutils.MotionlessTracker;
import frc.robot.visutils.MultiCamOdometryFactory;
import frc.robot.visutils.TurnToAngleHelper;
import frc.robot.visutils.VisionKalmanFilter;
import robotutils.pub.RobotUtilsFactory;
import robotutils.pub.interfaces.DriveSmoothInterface;
import robotutils.pub.interfaces.JoystickInputInterface;
import robotutils.pub.interfaces.JoystickInputsRecord;
import robotutils.pub.interfaces.PerRobotConfigInterface;
import robotutils.pub.interfaces.dashboard.DashboardManagerInterface;
import robotutils.pub.interfaces.dashboard.DashboardNames;
import robotutils.pub.interfaces.dashboard.Field2dObjectRenderer;
import robotutils.pub.interfaces.simio.ArmIoInterface;
import robotutils.pub.interfaces.simio.ElevatorIoInterface;
import robotutils.pub.interfaces.simio.RollerIoInterface;
import robotutils.pub.interfaces.simio.TwoMotorRollerIoInterface;

import java.util.Optional;
import java.util.OptionalDouble;


/**
 * The RobotContainer class is where the bulk of the robot structure is declared.
 *
 * <p>This class centralizes the hardware instances, UI device bindings, and
 * the mapping between operator inputs and subsystem commands for the 2026 season.
 */
public class RobotContainer {
    private final RobotUtilsFactory m_robotUtilsFactory = new RobotUtilsFactory();
    public final DashboardManagerInterface m_dashboardManager = m_robotUtilsFactory.createDashboardManager();

    private final PerRobotConfigInterface<BotConfigInterface> m_perRobotConfig = RobotConfigSelector.create(Optional.of(m_dashboardManager));
    private final BotConfigInterface m_configInterface = m_perRobotConfig.getBotConfig();

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
    private final DriveSmoothInterface m_driveSmooth = m_robotUtilsFactory.createDriveSmooth(
        DriveConstants.kTranslationSlewRate,
        DriveConstants.kRotationSlewRate,
        DriveConstants.kJoystickDeadband,
        DriveConstants.kTranslationExponent,
        DriveConstants.kRotationExponent);

    /** Processes raw joystick inputs into scaled robot velocities. */
    public final JoystickInputInterface m_joystickInput;

    /** The drivetrain subsystem, initialized using constants generated by Tuner X. */
    public final CommandSwerveDrivetrain drivetrain = m_configInterface.createDrivetrain();

    public final BasicInfoDashboard basicInfoDashboard;

    /** Field2d for Glass/SmartDashboard visualization. */
    public final Field2d m_glassField = new Field2d();

    /** Simulation wrapper - null when not in simulation. */
    public final SimWrapper m_simWrapper;
    public final ShowVisionOnField m_showVisionOnField;

    public final CamOdometryInterface m_multiCamlimelight;

    private final TwoMotorRollerIoInterface m_shooterIo =
        (Robot.isSimulation() || m_configInterface.shouldForceDisableShooter())
        ? SimIoFactory.createShooterIoSim()
        : new ShooterIoReal(m_configInterface);

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(
        m_configInterface,
        m_shooterIo);

    private final RollerIoInterface m_indexerIo =
        (Robot.isSimulation() || m_configInterface.shouldForceDisableIndexer())
        ? SimIoFactory.createIndexerIoSim()
        : new IndexerIoReal();

    private final RollerIoInterface m_spinnyIo =
        (Robot.isSimulation() || m_configInterface.shouldForceDisableSpinny())
        ? SimIoFactory.createSpinnyIoSim()
        : new frc.robot.subsystems.spinny.SpinnyIoReal();

    public final SpinnyWheels m_spinnyWheels = new SpinnyWheels(m_spinnyIo);

    public final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(m_indexerIo);

    private final ElevatorIoInterface m_climberIo =
        (Robot.isSimulation() || m_configInterface.shouldForceDisableClimber())
        ? SimIoFactory.createClimberIoSim()
        : new ClimberIoReal();

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem(m_climberIo);

    /** Intake IO: real hardware or FlywheelSim depending on mode. */
    private final RollerIoInterface m_intakeIo =
        (Robot.isSimulation() || m_configInterface.shouldForceDisableIntake())
        ? SimIoFactory.createIntakeIoSim()
        : new IntakeIoReal();

    private final ArmIoInterface m_intakeArmIo =
        (Robot.isSimulation() || m_configInterface.shouldForceDisableIntakeArm())
        ? SimIoFactory.createIntakeArmIoSim(ArmConstants.kMinArmAngle, ArmConstants.kMaxArmAngle)
        : new ArmIoReal();

    /** Intake subsystem driven through the IO abstraction. */
    public final IntakeSubsystem intakeSubsystem =
        new IntakeSubsystem(m_intakeIo);

    /** Arm subsystem driven through the IO abstraction. */
    public final ArmSubsystem armSubsystem = new ArmSubsystem(m_intakeArmIo);

    /** Hood subsystem using Actuonix linear actuator */
    public final HoodSubsystem hoodSubsystem = new HoodSubsystem();

    /** Vision-only Kalman filter for precise stationary position estimation. */
    public final VisionKalmanFilter m_visionKalmanFilter = new VisionKalmanFilter();

    /** Tracks whether the robot is motionless and for how long. */
    public final MotionlessTracker m_motionlessTracker;

    /**
     * Constructs the RobotContainer.
     * Initializes autonomous selection dashboards and binds controller inputs to commands.
     */
    public RobotContainer() {
        m_joystickInput = m_robotUtilsFactory.createJoystickInput(
            m_driveSmooth,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            driveController.rightBumper()::getAsBoolean,
            TeleoperatedSpeed,
            MaxAngularRate,
            Robot.isSimulation(),
            () -> drivetrain.getOperatorForwardDirection().getDegrees());

        m_simWrapper = SimWrapper.create(m_dashboardManager, m_configInterface, drivetrain, this::resetRobotPose);
        Command cycleResetCmd = (m_simWrapper != null)
            ? drivetrain.runOnce(() -> m_simWrapper.cycleResetPosition(
                AutoLogic.getSelectedAutoStartingPose()))
            : null;

        // We want to display objects from the ground truth simulation on Field2d
        if (m_simWrapper != null) {
            m_dashboardManager.addCustomRenderer(
                new Field2dObjectRenderer(
                    m_glassField,
                    "GroundTruthRobot"), // This is the name on the Field2d
                DashboardNames.kGroundTruthProviderName,
                DashboardNames.kGroundTruthPoseItemName);
        }

        // $TODO2 - Can I consolidate all the basicInfoDashboard config
        basicInfoDashboard = new BasicInfoDashboard(
            m_configInterface,
            drivetrain,
            m_glassField,
            cycleResetCmd,
            m_configInterface.getCameras().stream()
                .map(c -> c.cameraName)
                .collect(java.util.stream.Collectors.toList()));

        // Wire this up to avoid circular dependency
        m_visionKalmanFilter.setDashboard(basicInfoDashboard);

        m_showVisionOnField = new ShowVisionOnField(
            m_configInterface,
            m_glassField,
            (m_simWrapper == null) ? null : m_simWrapper.getSimDebugField());

        AutoLogic.initShuffleboard(drivetrain);

        configureDriveBindings();
        configureOperateBindings();
        configureDefaultCommands();
        registerNamedCommands();

        // Setup vision system
        m_motionlessTracker = MotionlessTracker.create(
            () -> drivetrain.getState().Speeds,
            m_visionKalmanFilter::reset);

        m_multiCamlimelight = MultiCamOdometryFactory.create(
            m_configInterface,
            new CamOutputs(
                info -> drivetrain.addVisionMeasurement(
                    info.pose(),
                    info.timestamp(),
                    info.estimationStdDevs()),
                m_visionKalmanFilter),
            new CamInputs(drivetrain::getState, drivetrain::samplePoseAt, m_motionlessTracker::isMotionless),
            basicInfoDashboard,
            m_motionlessTracker,
            m_configInterface.getEvaluatePosesName());
    }

    /** Registers named commands for PathPlanner */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("shoot", ShootCommand.create(shooterSubsystem, m_indexerSubsystem));

        NamedCommands.registerCommand("Full Auto Climb", FullAutoClimbCommand.create(climberSubsystem));

        NamedCommands.registerCommand("get fuel", GetFuelCommand.create(armSubsystem, intakeSubsystem));

        NamedCommands.registerCommand("set intake bottom", SetIntakeBottomCommand.create(armSubsystem, intakeSubsystem));

        NamedCommands.registerCommand("set intake top", SetIntakeTopCommand.create(armSubsystem, intakeSubsystem));

        if (Robot.isSimulation()) {

            // NOTE: Uses Commands.runOnce (NOT drivetrain.runOnce) intentionally — PathPlanner runs
            // named commands in parallel with FollowPathCommand, which already holds the drivetrain
            // subsystem. Taking a drivetrain requirement here would cause a subsystem conflict and
            // silently prevent the command from running.
            NamedCommands.registerCommand("nudge-right", Commands.runOnce(() ->
                SimWrapper.nudgeRight12Inches(m_simWrapper)));

            NamedCommands.registerCommand("nudge-rotate", Commands.runOnce(() ->
                SimWrapper.nudgeRotate45Degrees(m_simWrapper)));

            NamedCommands.registerCommand("faulty-pull-right", Commands.runOnce(() ->
                SimWrapper.enablePullRight(m_simWrapper, true)));

            NamedCommands.registerCommand("faulty-rotate-clockwise", Commands.runOnce(() ->
                SimWrapper.enableRotateClockwise(m_simWrapper, true)));

            NamedCommands.registerCommand("faulty-camera-misplaced", Commands.runOnce(() ->
                SimWrapper.enableCameraMisplaced(m_simWrapper, new Transform3d(0, -1.0, 0, new Rotation3d()))));
        }
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
        new Trigger(() -> JoyHelpers.isPovDownward(driveController)).whileTrue(
            new RotateToTargetCommand(drivetrain, () ->
                TurnToAngleHelper.getTag2dPose(m_multiCamlimelight.getPrimaryTagId())));

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

        operateController.povLeft().onTrue(hoodSubsystem.runOnce(() -> hoodSubsystem.setAngle(120)));
        operateController.povRight().onTrue(hoodSubsystem.runOnce(() -> hoodSubsystem.setAngle(0)));


        operateController.a().toggleOnTrue(new IntakeCommand(intakeSubsystem));

        // Right stick click: home the arm (must be done before any arm position commands)
        operateController.rightStick().onTrue(
            IntakeArmHomeCommand.create(armSubsystem));
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
                    JoyHelpers.isPovUpward(driveController),
                    m_multiCamlimelight.getPrimaryTagId(),
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
        if (Robot.isSimulation()) {
            return AutoLogic.getSelectedAutoCommand()
                .andThen(Commands.runOnce(() -> SimWrapper.resetAllAutoSimFaults(m_simWrapper)));
        }

        // Non-sim case:
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
    }

}
