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
import frc.robot.ramenlib.sim.SimConstants.SimCommandConstants;
import frc.robot.ramenlib.sim.simcommands.pretend.PretendCommandNoSystem;
import frc.robot.ramenlib.sim.simcommands.pretend.UnexpectedCommand;
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

    AutoLogic.initShuffleBoard();
    addTestButtonsToShuffleboard();

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
  }

  private Command CmdWrapperIntakeArmSystem(Command command) {
    // Now that we implemented sim for arm, re-enable these commands for sim.
    return command;
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
  
    //D-pad drives straight (no gyro) for tests
    // m_driverController.povCenter().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> 0, () -> 0, true)));
    // m_driverController.povUp().onTrue((m_swerveDrive.driveCommand(() -> -0.2, () -> 0, () -> 0, true)));
    // m_driverController.povDown().onTrue((m_swerveDrive.driveCommand(() -> 0.2, () -> 0, () -> 0, true)));
    // m_driverController.povLeft().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> 0.3, () -> 0, false)));
    // m_driverController.povRight().onTrue((m_swerveDrive.driveCommand(() -> 0, () -> -0.3, () -> 0, false)));
    

    // $TODO m_swerveDrive.sysIdDriveMotorCommand()
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
}