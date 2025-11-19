// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoNameConstants;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeSpitCommandConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OuttakeSpitCommandConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.DriveForwardNow;
import frc.robot.commands.OuttakeSpitCommand;
import frc.robot.ramenlib.sim.SimConstants.SimCommandConstants;
import frc.robot.ramenlib.sim.simcommands.pretend.PretendCommandNoSystem;
import frc.robot.ramenlib.sim.simcommands.pretend.PretendCommandOuttakeSystem;
import frc.robot.ramenlib.sim.simcommands.pretend.UnexpectedCommand;
import frc.robot.subsystems.OuttakeSystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AutoLogic;
import frc.robot.util.CommandAppliedController;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based
 * is a "declarative" paradigm, very little robot logic should actually be handled
 * in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger
 * mappings) should be declared here.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class RobotContainer
{
  private final CommandAppliedController m_driverController =
    new CommandAppliedController(OperatorConstants.kDriverPort);
  private final CommandAppliedController m_armController =
    new CommandAppliedController(OperatorConstants.kArmPort);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       m_swerveDrive  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                SwerveConstants.kJsonDirectory));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveDrive.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY(),
                                                                () -> m_driverController.getLeftX())
                                                            .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                                            .deadband(OperatorConstants.kDeadband)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command m_driveFieldOrientedDirectAngle = m_swerveDrive.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command m_driveFieldOrientedAngularVelocity = m_swerveDrive.driveFieldOriented(driveAngularVelocity);


  private final OuttakeSystem m_outtakeSystem = new OuttakeSystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // $TODO - Pancake robot doesnt have an arm intake system.  If we instantiate this on the 
    // pancake robot, the code will crash on startup, preventing code deployment from
    // succeeding.  Note that this fix is TEMPORARY to unblock pancake bot code deployment!
    // A better fix would be not to throw in IntakeArmSystem, at least not on pancakebot.

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    m_swerveDrive.initShuffleboard();
    AutoLogic.initShuffleBoard();
    addTestButtonsToShuffleboard();

    NamedCommands.registerCommand("Outtake from Bucket", CmdWrapperOuttakeSystem(new OuttakeSpitCommand(m_outtakeSystem, OuttakeSpitCommandConstants.speed).withTimeout(1)));

    NamedCommands.registerCommand("Align to April Tag Left Side", CmdWrapperUnexpectedCommand(m_swerveDrive.alignWithAprilTagCommand(
      AlignRobotConstants.transformDrive,
      AlignRobotConstants.transformLeftStrafe
    ), "alignAprilLeft"));
    NamedCommands.registerCommand("Align to April Tag Right Side", CmdWrapperUnexpectedCommand(m_swerveDrive.alignWithAprilTagCommand(
      AlignRobotConstants.transformDrive,
      AlignRobotConstants.transformRightStrafe
    ), "alignAprilRight"));

    //
    // Heres the Commands that we dont mock in simulation, since they work just fine in sim.
    //[]\


    //NamedCommands.registerCommand("waitFiveSeconds", waitFiveSeconds);
  }

  private Command printStartStop(Command command, String name) {
    return command.beforeStarting(() -> System.out.println(">--> " + name + " start"))
                  .andThen(() -> System.out.println(">--> " + name + " end"));
  }

  private void addTestButtonsToShuffleboard() {
    ShuffleboardTab tabTest = Shuffleboard.getTab("Test");

    Command spinCmd = printStartStop(
      m_swerveDrive.sysIdDriveMotorCommand(true), "Spin test");
    tabTest.add("Spin Test", spinCmd).withWidget("Command");

    Command driveCmd = printStartStop(
      m_swerveDrive.sysIdDriveMotorCommand(false), "Drive test");
    tabTest.add("Drive Test", driveCmd).withWidget("Command");

    Command angleTest = printStartStop(
      m_swerveDrive.sysIdAngleMotorCommand(), "Angle test");
    tabTest.add("Angle Test", angleTest).withWidget("Command");
  }

  private Command CmdWrapperIntakeArmSystem(Command command) {
    // Now that we implemented sim for arm, re-enable these commands for sim.
    return command;
  } 

  private Command CmdWrapperOuttakeSystem(Command command) {
    if (disableCommandsInSim()) {
      return new PretendCommandOuttakeSystem(m_outtakeSystem);
    } else {
      return command;
    }
  }

  private Command CmdWrapperNoSystem(Command command) {
    if (disableCommandsInSim()) {
      return new PretendCommandNoSystem();
    } else {
      return command;
    }
  }

  private Command CmdWrapperUnexpectedCommand(Command command, String name) {
    if (disableCommandsInSim()) {
      return new UnexpectedCommand(name);
    } else {
      return command;
    }
  }

  // In simulation, not all Subsystems are simulated yet.  Therefore, in simulation,
  // we disable most registerCommand() that pathplanner calls, so that those commands
  // don't do anything. That lets us run the simulation for auto for the subsystems
  // that ARE currently simulated, such as Swerve.
  private boolean disableCommandsInSim() {
    if (!RobotBase.isSimulation()) {
      return false;
    }

    return SimCommandConstants.kDisableMostCommandsInSim;
  }

  public void configureDefaultCommands() {
    m_swerveDrive.setDefaultCommand(m_driveFieldOrientedAngularVelocity);
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    /*
        m_driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
        m_driverController.b().whileTrue(
            Commands.deferredProxy(() -> drivebase.driveToPose(
                                        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                                  ));
        m_driverController.y().whileTrue(drivebase.aimAtSpeaker(2));
        m_driverController.start().whileTrue(Commands.none());
        m_driverController.back().whileTrue(Commands.none());
        m_driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        m_driverController.rightBumper().onTrue(Commands.none());
    */
    //this is field relative, right stick controls orientation relative to the field
    //drivebase.setDefaultCommand(m_driveFieldOrientedDirectAngle);



    // this is field relative, right stick controls rotation around z axis
    configureDefaultCommands();
  
  
    //D-pad drives straight (no gyro) for tests
    // m_driverController.povCenter().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> 0, () -> 0, true)));
    // m_driverController.povUp().onTrue((m_swerveDrive.driveCommand(() -> -0.2, () -> 0, () -> 0, true)));
    // m_driverController.povDown().onTrue((m_swerveDrive.driveCommand(() -> 0.2, () -> 0, () -> 0, true)));
    // m_driverController.povLeft().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> 0.3, () -> 0, false)));
    // m_driverController.povRight().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> -0.3, () -> 0, false)));
    

    // $TODO m_swerveDrive.sysIdDriveMotorCommand()

    // Start button resets the gyro
    m_driverController.start().onTrue((Commands.runOnce(m_swerveDrive::zeroGyroWithAlliance)));

    // A button aligns the robot using the AprilTag
    //m_driverController.a().onTrue(new AimAtLimeLightV2(m_swerveDrive));
    m_driverController.povLeft().onTrue(m_swerveDrive.alignWithAprilTagCommand(
    AlignRobotConstants.transformDrive,
    AlignRobotConstants.transformLeftStrafe
  ));
    m_driverController.povRight().onTrue(m_swerveDrive.alignWithAprilTagCommand(
      AlignRobotConstants.transformDrive,
      AlignRobotConstants.transformRightStrafe
  ));
    
    // Outtake reverse
    m_armController.povLeft().whileTrue(new OuttakeSpitCommand(m_outtakeSystem, -OuttakeSpitCommandConstants.speed));
    // Outtake coral
    m_armController.povRight().whileTrue(new OuttakeSpitCommand(m_outtakeSystem, OuttakeSpitCommandConstants.speed));
    
    // 
    m_armController.start().whileTrue(new OuttakeSpitCommand(m_outtakeSystem, -OuttakeSpitCommandConstants.speed));

    new Trigger(() -> (m_driverController.leftBumper().getAsBoolean() && m_swerveDrive.getVisionSystem().isDetecting())).onTrue(
      Commands.runOnce(() -> m_swerveDrive.trueResetPose())
    );

  }

  //private Command waitFiveSeconds = new WaitCommand(5)
    //.beforeStarting(() -> System.out.println("Auto wait 5 seconds start"))
    //.andThen(() -> System.out.println("Auto end wait"));

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    //This is the Autologic that is used through Path Planner
    return AutoLogic.getAutoCommand(AutoLogic.autoPicker.getSelected());

    // This is the alternative Autologic code that is manually coded
    //switch (AutoLogic.autoPicker.getSelected()) {
      //case AutoNameConstants.kCenterL1AutoName:
        //return new DriveForwardNow(m_swerveDrive, 1.4, true)
        //.andThen(CmdWrapperIntakeArmSystem(new SetArmToAngleCommand(m_armSystem, ArmConstants.L1ArmAngle)))
        //.andThen(CmdWrapperIntakeSystem(new IntakeSpitCommand(m_intakeSystem, IntakeSpitCommandConstants.speed, true)));

      //case AutoNameConstants.kCenterL4AutoName:
          //return new DriveForwardNow(m_swerveDrive, 1, false)
          //.andThen(CmdWrapperElevatorSystem(new ElevatorToPositionCommand(m_elevatorSystem, ElevatorConstants.kMaxElevatorPosition).withTimeout(2)))
          //.andThen(new DriveForwardNow(m_swerveDrive, 0.5, false))
          //.andThen(CmdWrapperOuttakeSystem(new OuttakeSpitCommand(m_outtakeSystem, OuttakeSpitCommandConstants.speed, true)));
      
      //default:
        //return new DriveForwardNow(m_swerveDrive, 1.4, true).
          //andThen(CmdWrapperIntakeArmSystem(new SetArmToAngleCommand(m_armSystem, ArmConstants.L1ArmAngle)))
          //.andThen(CmdWrapperIntakeSystem(new IntakeSpitCommand(m_intakeSystem, IntakeSpitCommandConstants.speed, true)));
  }
  
  public void setMotorBrake(boolean brake)
  {
    m_swerveDrive.setMotorBrake(brake);
  }

  public void updateVisionPose() {
    m_swerveDrive.updateVisionPose();
  }
}