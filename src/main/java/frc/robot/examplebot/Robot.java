// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
// $TODO - Basic vision and show debug field
import frc.robot.sim.VisionSimFactory;
import frc.robot.sim.VisionSimInterface;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final VisionSimInterface m_visionSim;

    public Robot() {
        m_robotContainer = new RobotContainer();

        // $TODO - Basic vision and show debug field
        m_visionSim = VisionSimFactory.create();
        m_visionSim.subscribePoseEstimates(m_robotContainer.drivetrain::addVisionMeasurement);

        // $TODO - Clean reset
        // Set the vision resetter so pose resets also reset vision simulation
        m_robotContainer.setVisionResetter(m_visionSim::resetSimPose);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        // Update vision simulation (processes camera results and updates pose estimator)
        // $TODO - Basic vision and show debug field
        if (m_visionSim != null) {
            m_visionSim.periodic();
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        var driveState = m_robotContainer.drivetrain.getState();
        // $TODO - Basic vision and show debug field
        var robotPoseHoldingCamera = driveState.Pose;

        // $TODO - Ground truth
        if (m_robotContainer.groundTruthSim != null) {
            m_robotContainer.groundTruthSim.simulationPeriodic();

            // Update vision simulation with the ground truth robot pose (from physics)
            // This ensures the camera sees AprilTags based on where the robot actually is,
            // not where odometry thinks it is. This allows testing of vision correction.
            robotPoseHoldingCamera = m_robotContainer.groundTruthSim.getGroundTruthPose();
        }

        // $TODO - Basic vision and show debug field
        if (m_visionSim != null) {
            m_visionSim.simulationPeriodic(robotPoseHoldingCamera);
        }

        // $TODO - Basic vision and show debug field
        var debugField = m_visionSim != null ? m_visionSim.getSimDebugField() : null;
        if (debugField != null) {
            // Show the estimated pose (what odometry thinks)
            debugField.getObject("EstimatedRobot").setPose(driveState.Pose);
            debugField.getObject("EstimatedRobotModules").setPoses(getModulePoses(driveState));

            if (m_robotContainer.groundTruthSim != null) {
                // Show the ground truth pose (where the robot actually is in simulation)
                debugField.getObject("GroundTruthRobot").setPose(
                    m_robotContainer.groundTruthSim.getGroundTruthPose());
            }
        }
    }

    /**
     * Get the Pose2d of each swerve module based on the current robot pose and module states.
     */
    // $TODO - Basic vision and show debug field
    private Pose2d[] getModulePoses(com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState driveState) {
        // Module locations relative to robot center (from TunerConstants)
        Translation2d[] moduleLocations = {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };

        Pose2d[] modulePoses = new Pose2d[4];
        for (int i = 0; i < 4; i++) {
            modulePoses[i] = driveState.Pose.transformBy(
                new Transform2d(moduleLocations[i], driveState.ModuleStates[i].angle)
            );
        }
        return modulePoses;
    }
}
