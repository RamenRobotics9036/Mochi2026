// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.vision.VisionConstants;
import frc.robot.botconfig.CameraConfig;
import frc.robot.botconfig.RobotIdentity;
import frc.robot.commands.FullAutoClimbCommand;
import frc.robot.commands.GetFuelCommand;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.commands.IntakeArmHomeCommand;
import frc.robot.commands.SetIntakeBottomCommand;
import frc.robot.commands.SetIntakeTopCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.commands.SpinnyDefaultCommand;
import frc.robot.sim.JoystickInputsRecord;
import frc.robot.sim.RollerSim.RollerIoInterface;
import frc.robot.sim.RollerSim.TwoMotorRollerIoInterface;
import frc.robot.sim.SimIoFactory;
import frc.robot.sim.elevatorSim.ElevatorIoInterface;
import frc.robot.subsystems.climber.ClimberIoReal;
import frc.robot.sim.armsim.ArmIoInterface;
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
import frc.robot.vision.CameraWrapper;
import frc.robot.LimelightHelpers;
import frc.robot.vision.VisionSubsystem;
import java.util.List;
import java.util.stream.Collectors;

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

    /** Standard field-centric swerve request. Uses Velocity control for smoother movement. */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.001 * TeleoperatedSpeed)
            .withRotationalDeadband(0.001 * MaxAngularRate)
            .withDriveRequestType(DriveRequestType.Velocity);

    // Request used for both Static Aiming and Shoot on the Move (Robot-Centric tracking)
    private final SwerveRequest.RobotCentric robotCentricAimRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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


    /** Processes raw joystick inputs into scaled robot velocities. */
    public final JoystickInput m_joystickInput;

    /** The drivetrain subsystem, initialized using constants generated by Tuner X. */
    public final CommandSwerveDrivetrain drivetrain = m_configInterface.createDrivetrain();

    /** Field2d for Glass/SmartDashboard visualization. */
    public final Field2d m_glassField = new Field2d();

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
        ? SimIoFactory.createIntakeArmIoSim()
        : new ArmIoReal();

    /** Intake subsystem driven through the IO abstraction. */
    public final IntakeSubsystem intakeSubsystem =
        new IntakeSubsystem(m_intakeIo);

    /** Arm subsystem driven through the IO abstraction. */
    public final ArmSubsystem armSubsystem = new ArmSubsystem(m_intakeArmIo);

    /** Hood subsystem using Actuonix linear actuator */
    public final HoodSubsystem hoodSubsystem = new HoodSubsystem();

    /** Vision subsystem: processes all Limelight cameras and fuses into odometry */
    public final VisionSubsystem visionSubsystem;

    public RobotContainer() {
        m_joystickInput = new JoystickInput(
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            driveController.rightBumper()::getAsBoolean,
            TeleoperatedSpeed,
            MaxAngularRate,
            Robot.isSimulation(),
            () -> drivetrain.getOperatorForwardDirection().getDegrees());

        AutoLogic.initShuffleboard(drivetrain);

        // Build VisionSubsystem from the bot config's camera list
        List<CameraConfig> camCfgs = m_configInterface.getCameras();
        List<CameraWrapper> cameras = camCfgs.stream()
            .map(cfg -> new CameraWrapper(cfg.name(), cfg.robotToCamera()))
            .collect(Collectors.toList());
        visionSubsystem = new VisionSubsystem(cameras, drivetrain);

        configureDriveBindings();
        configureOperateBindings();
        configureDefaultCommands();
        registerNamedCommands();
    }

    /** Registers named commands for PathPlanner */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("shoot", ShootCommand.create(shooterSubsystem, m_indexerSubsystem));

        NamedCommands.registerCommand("Full Auto Climb", FullAutoClimbCommand.create(climberSubsystem));

        NamedCommands.registerCommand("get fuel", GetFuelCommand.create(armSubsystem, intakeSubsystem));

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

        // Hook up the telemetry logger to the drivetrain periodic updates
        drivetrain.registerTelemetry(logger::telemeterize);

        // Hold 'A' Button: Shoot on the Move
        // Manual stick strafe, automatic target heading and distance tracking
        driveController.a().whileTrue(
            drivetrain.applyRequest(() -> robotCentricAimRequest
                .withVelocityX(getLimelightForwardSpeed())     // Auto-distance
                .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Manual strafe
                .withRotationalRate(getLimelightRotationSpeed()) // Auto-heading
            )
        );

        // Hold 'Y' Button: Static Aim and Range
        // Complete hands-off control. The robot automatically drives to the target and aligns
        driveController.y().whileTrue(
            drivetrain.applyRequest(() -> robotCentricAimRequest
                .withVelocityX(getLimelightForwardSpeed())
                .withVelocityY(0.0)
                .withRotationalRate(getLimelightRotationSpeed())
            )
        );
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
     * Returns the name of the camera with the best target lock.
     * Iterates over all configured cameras, picks the one with a valid target
     * and smallest |TX| (closest to center). Falls back to "limelight".
     */
    private String getBestAimCamera() {
        String fallback = "limelight";
        double bestTx = Double.MAX_VALUE;

        if (visionSubsystem != null) {
            for (CameraWrapper cam : visionSubsystem.getCameras()) {
                String name = cam.getName();
                if (LimelightHelpers.getTV(name)) {
                    double tx = Math.abs(LimelightHelpers.getTX(name));
                    if (tx < bestTx) {
                        bestTx = tx;
                        fallback = name;
                    }
                }
            }
        } else {
            // Fallback to default name if no cameras configured
            if (LimelightHelpers.getTV("limelight")) {
                return "limelight";
            }
        }

        return fallback;
    }

    // Used by 'A' and 'Y' Buttons to control rotation
    private double getLimelightRotationSpeed() {
        String cam = getBestAimCamera();
        if (!LimelightHelpers.getTV(cam)) {
            return 0.0;
        }

        double tx = LimelightHelpers.getTX(cam);
        if (Math.abs(tx) < VisionConstants.AIM_ROT_DEADBAND_DEG) {
            return 0.0;
        }

        // Note: If the robot still spins rapidly when tracking,
        // change "-MaxAngularRate" to "MaxAngularRate" to invert the direction.
        return tx * VisionConstants.AIM_ROT_KP * -MaxAngularRate;
    }

    // Used by 'A' and 'Y' Buttons to control forward/backward distance
    private double getLimelightForwardSpeed() {
        String cam = getBestAimCamera();
        if (!LimelightHelpers.getTV(cam)) {
            return 0.0;
        }

        double currentTY = LimelightHelpers.getTY(cam);
        double error = currentTY - VisionConstants.AIM_TY_SETPOINT;

        if (Math.abs(error) < VisionConstants.AIM_FWD_DEADBAND_DEG) {
            return 0.0;
        }

        double targetingForwardSpeed = error * VisionConstants.AIM_FWD_KP * MaxSpeed * -1.0;
        return edu.wpi.first.math.MathUtil.clamp(
            targetingForwardSpeed,
            -MaxSpeed * VisionConstants.AIM_MAX_FWD_FRACTION,
            MaxSpeed * VisionConstants.AIM_MAX_FWD_FRACTION);
    }

    /**
     * Retrieves the autonomous routine selected via the dashboard at the start of the match.
     *
     * @return The command to execute during the autonomous period.
     */
    public Command getAutonomousCommand() {
        return AutoLogic.getSelectedAutoCommand();
    }


}
