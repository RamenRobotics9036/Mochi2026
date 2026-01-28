// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.auto.AutoLogic;

/**
 * The main robot class that controls the flow of the 2026 FRC robot code.
 * 
 * <p>This class manages the lifecycle of the robot (Initialization, Autonomous, 
 * Teleop, and Test modes) and coordinates with the CommandScheduler to run commands.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * Initializes the RobotContainer, which sets up all subsystem hardware, 
   * button bindings, and default commands.
   */
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * Called once when the robot boots up.
   * Configures global logging settings and prepares autonomous path-following commands.
   */
  @Override
  public void robotInit() {
    // Start WPILib's internal data logging system for post-match analysis
    DataLogManager.start();
    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(DataLogManager.getLog());

    // Pre-calculate and cache PathPlanner paths to avoid CPU spikes during matches
    AutoLogic.registerCommands();
  }

  /**
   * Called every 20ms regardless of mode.
   * Runs the CommandScheduler, which handles all active command execution.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  /**
   * Called once when the autonomous period begins.
   * Retrieves the selected routine from the dashboard and schedules it for execution.
   */
  @Override
  public void autonomousInit() {
    Shuffleboard.startRecording();

    // Fetch the specific command selected by the drive team via the Auto Chooser
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Execute the autonomous routine
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  /**
   * Called once when the teleoperated period begins.
   * Ensures the autonomous routine is stopped before the driver takes control.
   */
  @Override
  public void teleopInit() {
    // Stop autonomous to prevent conflicts with manual driver inputs
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Shuffleboard.startRecording();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  /**
   * Called once when Test mode is enabled via the Driver Station.
   * Clears all running commands to provide a clean state for debugging.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /**
   * Optional simulation periodic loop. 
   * This is where physics engine updates would be called if running on a PC.
   */
  @Override
  public void simulationPeriodic() {}
}
