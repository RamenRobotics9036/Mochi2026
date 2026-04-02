// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.ShowVisionOnField;
import frc.robot.subsystems.auto.AutoLogic;
import frc.robot.visutils.PerCycleState;
import frc.robot.visutils.VisionKalmanFilter.DisplayInfo;

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
  @SuppressWarnings("VariableDeclarationUsageDistance")
  @Override
  public void robotPeriodic() {
    // Update motionless tracking early - this resets Kalman filter when robot moves
    m_robotContainer.m_motionlessTracker.update();

    // $VISIONSIM - Wrapper for sim features
    if (Robot.isSimulation() && m_robotContainer.m_simWrapper != null) {
        // NOTE: We run the vision period FIRST in robotPeriodic, since it updates
        // NetworkTables with the limelight data, in-case any code in this loop
        // needs that info and doesnt want it delayed 20ms.
        m_robotContainer.m_simWrapper.robotPeriodic();
    }

    // We allow vision to be enabled/disabled DYNAMICALLY from dashboard, so we set whether
    // its enabled on EACH cycle.
    m_robotContainer.m_multiCamlimelight.enableVision(
        m_robotContainer.basicInfoDashboard.isVisionEnabled());

    m_robotContainer.m_multiCamlimelight.periodic();

    Optional<Pose2d> showVisPose =
      m_robotContainer.m_multiCamlimelight.getEstimatedPose();

    DisplayInfo kalmanDisplay = m_robotContainer.m_visionKalmanFilter.getFieldDisplayInfo(0.4);

    CommandScheduler.getInstance().run();

    SwerveDriveState driveState = m_robotContainer.drivetrain.getState();

    // Take field info gathered, and show it on dashboard
    m_robotContainer.m_showVisionOnField.updateFieldDisplay(
        showVisPose,
        kalmanDisplay,
        driveState);

    // We update logging after CommandScheduler.run(), so that any commands that
    // changed drivetrain state are reflected in the telemetry.
    if (m_robotContainer.basicInfoDashboard != null) {
        m_robotContainer.basicInfoDashboard.update();
    }
    m_robotContainer.m_dashboardManager.update();
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
    m_robotContainer.resetDriveSmoothing();

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
    m_robotContainer.resetDriveSmoothing();

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
  public void simulationPeriodic() {
    // $VISIONSIM - Wrapper for sim features
    if (m_robotContainer.m_simWrapper != null) {
        m_robotContainer.m_simWrapper.simulationPeriodic();

        // Debug field visualization
        Pose2d groundTruthPose = m_robotContainer.m_simWrapper.getGroundTruthPose();
        m_robotContainer. m_showVisionOnField.showGroundTruthPoseOnField(groundTruthPose);
    }
  }
}
