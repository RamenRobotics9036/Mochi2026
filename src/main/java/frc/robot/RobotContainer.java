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
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.botconfig.RobotIdentity;
import frc.robot.commands.FullAutoClimbCommand;
import frc.robot.commands.GetFuelCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateToTargetCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.sim.JoystickInputsRecord;
import frc.robot.sim.RollerSim.RollerIoInterface;
import frc.robot.sim.RollerSim.RollerIoSim;
import frc.robot.sim.RollerSim.TwoMotorRollerIoInterface;
import frc.robot.sim.RollerSim.TwoMotorRollerIoSim;
import frc.robot.sim.ShowVisionOnField;
import frc.robot.sim.elevatorSim.ElevatorIoInterface;
import frc.robot.sim.elevatorSim.ElevatorIoSim;
import frc.robot.subsystems.climber.ClimberIoReal;
import frc.robot.sim.SimWrapper;
import frc.robot.sim.armsim.ArmIoInterface;
import frc.robot.sim.armsim.ArmIoSim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpinnyWheels;
import frc.robot.subsystems.TestSubsystems;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.subsystems.indexer.IndexerIoReal;
import frc.robot.subsystems.intake.ArmIoReal;
import frc.robot.subsystems.intake.IntakeIoReal;
import frc.robot.subsystems.shooter.ShooterIoReal;
import frc.robot.visutils.AimController;
import frc.robot.visutils.DriveAccuracyTester;
import frc.robot.visutils.DriveSmooth;
import frc.robot.visutils.SingleCamOdometry;
import frc.robot.visutils.MotionlessTracker;
import frc.robot.visutils.MultiCamOdometry;
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
        ? new TwoMotorRollerIoSim(
            Constants.SimShooterConstants.kDeviceName,
            Constants.SimShooterConstants.kMoiKgM2,
            Constants.ShooterConstants.kShooterGearRatio)
        : new ShooterIoReal(m_configInterface);

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(
        m_configInterface,
        m_shooterIO);

    private final RollerIoInterface m_indexerIO = Robot.isSimulation()
        ? new RollerIoSim(
            Constants.SimIndexerConstants.kDeviceName,
            Constants.SimIndexerConstants.kMoiKgM2,
            Constants.IndexerConstants.kIndexerGearRatio)
        : new IndexerIoReal();

    private final RollerIoInterface m_spinnyIO = Robot.isSimulation()
        ? new RollerIoSim(
            Constants.SimSpinnyWheelsConstants.kDeviceName,
            Constants.SimSpinnyWheelsConstants.kMoiKgM2,
            Constants.SpinnyWheelsConstants.kSpinGearRatio)
        : new frc.robot.subsystems.spinny.SpinnyIoReal();

    public final SpinnyWheels m_spinnyWheels = new SpinnyWheels(m_spinnyIO);

    public final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(m_indexerIO);

    private final ElevatorIoInterface m_climberIO = Robot.isSimulation()
        ? new ElevatorIoSim(
            Constants.SimClimberConstants.kDeviceName,
            Constants.SimClimberConstants.kGearRatio,
            Constants.SimClimberConstants.kCarriageMassKg,
            Constants.SimClimberConstants.kDrumRadiusMeters,
            Constants.SimClimberConstants.kMinHeightMeters,
            Constants.SimClimberConstants.kMaxHeightMeters)
        : new ClimberIoReal();

    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem(m_climberIO);

    /** Intake IO: real hardware or FlywheelSim depending on mode. */
    private final RollerIoInterface m_intakeIO = Robot.isSimulation()
        ? new RollerIoSim(
            Constants.SimIntakeConstants.kDeviceName,
            Constants.SimIntakeConstants.kMoiKgM2,
            Constants.IntakeConstants.kIntakeRollerGearRatio)
        : new IntakeIoReal();

    private final ArmIoInterface m_intakeArmIO = Robot.isSimulation()
        ? new ArmIoSim(
            Constants.SimIntakeArmConstants.kDeviceName,
            Constants.SimIntakeArmConstants.kMoiKgM2,
            Constants.SimIntakeArmConstants.kArmLengthMeters,
            Constants.ArmConstants.kArmGearRatio)
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

        // Put some debug info on the dashboard
        SmartDashboard.putString(
            "MAC Address Name",
            RobotIdentity.getBotName());
        SmartDashboard.putString(
            "Robot Config",
            m_configInterface.getConfigName());

        Field2d debugField = null;

        SmartDashboard.putData("GlassField", m_glassField);

        AutoLogic.initShuffleboard(drivetrain);
        m_driveAccuracyTester = new DriveAccuracyTester(
            drivetrain, m_visionKalmanFilter, basicInfoDashboard::forceDisableVision);

        // Add a button on dashboard to launch Accuracy Drive Test
        SmartDashboard.putData("Accuracy Drive Test", m_driveAccuracyTester.createTapeDropAutoCommand());

        SmartDashboard.putData("Test Subsystems", TestSubsystems.test(
            intakeSubsystem,
            m_indexerSubsystem,
            shooterSubsystem,
            armSubsystem,
            climberSubsystem));

        configureBindings();

        registerNamedCommands();

        m_spinnyWheels.setDefaultCommand(new RunCommand(m_spinnyWheels::spin, m_spinnyWheels));

        // $VISIONSIM - Wrapper for sim features
        if (Robot.isSimulation()) {
            m_simWrapper = new SimWrapper(
                m_configInterface,
                drivetrain,
                this::resetRobotPose);

            debugField = m_simWrapper.getSimDebugField();
        }
        else {
            m_simWrapper = null;
        }

        m_showVisionOnField = new ShowVisionOnField(
            m_configInterface,
            m_glassField,
            debugField);

        // Setup vision system
        m_motionlessTracker = new MotionlessTracker(() -> drivetrain.getState().Speeds);
        m_motionlessTracker.setOnStartedMoving(m_visionKalmanFilter::reset);

        m_multiCamlimelight = new MultiCamOdometry(
            m_configInterface,
            drivetrain::addVisionMeasurement);
        m_multiCamlimelight.setVisionDependencies(
            basicInfoDashboard::isVisionEnabled,
            m_visionKalmanFilter,
            m_motionlessTracker::isMotionless);

        basicInfoDashboard.setVisionDependencies(
            m_multiCamlimelight::getCurrentConfidenceScore,
            m_multiCamlimelight::getNumLockedTags,
            m_multiCamlimelight::getTx,
            m_multiCamlimelight::getTargetList,
            m_motionlessTracker::isMotionless,
            m_motionlessTracker::getSecondsStill);

        basicInfoDashboard.setVisionKalmanSupplier(() -> m_visionKalmanFilter);
    }

    private void registerNamedCommands() {
        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("shoot", ShootCommand.create(shooterSubsystem, m_indexerSubsystem));

        NamedCommands.registerCommand("Full Auto Climb", FullAutoClimbCommand.create(climberSubsystem));

        NamedCommands.registerCommand("get fuel", GetFuelCommand.create(armSubsystem, intakeSubsystem));

        NamedCommands.registerCommand("set intake bottom",
                new RunCommand(() -> armSubsystem.setArmPosition(Constants.ArmConstants.kMaxArmAngle),
                        intakeSubsystem)
                                .until(armSubsystem::isArmDeployed));

        NamedCommands.registerCommand("set intake top",
                new RunCommand(() -> armSubsystem.setArmPosition(Constants.ArmConstants.kMinArmAngle),
                        intakeSubsystem)
                                .withTimeout(2.0)
                                .andThen(armSubsystem::stop));
    }

    /**
     * Defines trigger-to-command mappings.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                JoystickInputsRecord inputs = m_joystickInput.getJoystickInputs();

                // POV-UP held: auto-aim rotation at the tag
                // while keeping X/Y translation.
                var driveState = drivetrain.getState();
                OptionalDouble aimRate = m_aimController.update(
                    isLeftPovUpward(),
                    m_multiCamlimelight.getLastTarget(),
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
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Try to align to an AprilTag with the Left Trigger
        driveController.leftTrigger().onTrue(drivetrain.AlignToTag(
            m_configInterface,
            driveController,
            MaxAngularRate));

        // $VISIONSIM - POV buttons for sim
        if (Robot.isSimulation()) {
            // In simulation, inject drift with POV right to test vision correction
            driveController.povRight()
                .onTrue(drivetrain.runOnce(() -> m_simWrapper.injectDrift(0.5, 15.0)));

            // POV left resets robot to the starting pose of the selected auto
            driveController.povLeft().onTrue(
                drivetrain
                    .runOnce(() -> m_simWrapper.cycleResetPosition(AutoLogic.getSelectedAutoStartingPose())));
        }

        // Hook up the telemetry logger to the drivetrain periodic updates
        drivetrain.registerTelemetry(logger::telemeterize);

        // POV Down: rotate in place to face the configured AprilTag.
        // Use a tolerant trigger (135°–225°) instead of exact povDown() (180° only)
        // to avoid command cancellation from D-pad diagonal flicker.
        new Trigger(this::isLeftPovDownward).whileTrue(
            new RotateToTargetCommand(drivetrain, () ->
                TurnToAngleHelper.getTag2dPose(m_multiCamlimelight.getLastTarget())));

        operateController.a().toggleOnTrue(new IntakeCommand(intakeSubsystem));

        // Run the intake arm manually any time the operator moves the right stick.
        // (Deadband prevents scheduling from tiny stick noise.)
        new Trigger(() -> Math.abs(operateController.getRightY()) > DriveConstants.kJoystickDeadband)
            .whileTrue(new IntakeArmCommand(armSubsystem, operateController));
    }

    /**
     * Wrapper for team-specific commands to handle simulation behavior.
     *
     * @param command The command to wrap.
     * @return The original command or an empty command if in simulation.
     */
    private Command CmdWrapperTeamCommand(Command command) {
        if (RobotBase.isSimulation()) {
            return Commands.none();
        }
        return command;
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

    /** Returns {@code true} when the left D-pad is in the downward region (135°–225°). */
    private boolean isLeftPovDownward() {
        int pov = driveController.getHID().getPOV();
        return pov != -1 && (pov >= 135 && pov <= 225);
    }

    /** Returns {@code true} when the left D-pad is in the upward region (315°–360° or 0°–45°). */
    private boolean isLeftPovUpward() {
        int pov = driveController.getHID().getPOV();
        return pov != -1 && (pov <= 45 || pov >= 315);
    }
}
