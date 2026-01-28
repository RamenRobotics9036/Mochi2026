// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Resets the robot's odometry to match the global pose estimated by the vision system.
 * 
 * <p>This "Snap" command uses Limelight's MegaTag2 algorithm to reconcile accumulated
 * wheel slip and gyro drift. It is designed to be highly reliable by enforcing distance
 * and visibility filters before allowing an odometry reset.
 * 
 * <p><b>Strategy:</b> Trigger this before high-precision automated tasks (like auto-aiming)
 * or after collisions to ensure the robot "knows" exactly where it is on the field.
 */
public class SnapToPoseCommand extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private boolean m_snapSucceeded = false;

    /**
     * Creates a new SnapToPoseCommand.
     * 
     * @param drivetrain The {@link CommandSwerveDrivetrain} instance to be updated.
     */
    public SnapToPoseCommand(CommandSwerveDrivetrain drivetrain) {
        // Store drivetrain reference
        m_drivetrain = drivetrain;
        
        // Note: We do NOT call addRequirements(m_drivetrain) here.
        // This allows the driver to continue driving/strafing while the pose
        // is updated in the background without interrupting the default drive command.
    }

    /**
     * Executes the pose-synchronization logic.
     * 
     * Pulls the MegaTag2 pose estimate, validates it against predefined safety 
     * thresholds (tag count and distance), and updates the drivetrain odometry.
     */
    @Override
    public void initialize() {
        m_snapSucceeded = false;
        
        try {
            // Retrieve pose using MegaTag2 (optimized for robots in motion using gyro data)
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                VisionConstants.kFixedCameraName
            );

            // Validation 1: Ensure the camera is actually returning data
            if (poseEstimate == null) {
                reportFailure("No pose estimate available (Camera disconnected?)");
                return;
            }

            // Validation 2: Ensure enough tags are visible to minimize "ghost" poses
            if (poseEstimate.tagCount < VisionConstants.MIN_TAG_COUNT) {
                reportFailure("Insufficient tags: " + poseEstimate.tagCount);
                return;
            }

            // Validation 3: Ensure tags are close enough that depth error is negligible
            if (poseEstimate.avgTagDist > VisionConstants.SNAP_MAX_DISTANCE) {
                reportFailure("Tags beyond distance threshold: " + String.format("%.2f", poseEstimate.avgTagDist) + "m");
                return;
            }

            // Success: Overwrite internal odometry with the vision-validated pose
            Pose2d visionPose = poseEstimate.pose;
            m_drivetrain.resetPose(visionPose);
            
            m_snapSucceeded = true;
            reportSuccess(visionPose, poseEstimate.tagCount, poseEstimate.avgTagDist);

        } catch (Exception e) {
            // Prevent code crashes if the Limelight helper encounters unexpected networking issues
            reportFailure("Vision System Exception: " + e.getMessage());
        }
    }

    /**
     * This is an instant command. It returns true immediately after {@link #initialize()} finishes.
     */
    @Override
    public boolean isFinished() {
        // Command completes immediately after initialization
        return true;
    }

    /**
     * Allows pose snapping during the pre-match period or while testing on blocks.
     */
    @Override
    public boolean runsWhenDisabled() {
        // Allows alowed operation when robot is disabled
        return true;
    }

    /**
     * @return True if the most recent execution successfully updated the robot's pose.
     */
    public boolean snapSucceeded() {
        // was the snap successful?
        return m_snapSucceeded;
    }

    /**
     * Logs successful synchronization to the Driver Station for operator awareness.
     */
    private void reportSuccess(Pose2d pose, int tagCount, double avgDist) {
        String message = String.format(
            "SNAP SUCCESS: Pose=(%.2f, %.2f, %.1f°) | Tags=%d | AvgDist=%.2fm",
            pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
            tagCount, avgDist
        );
        System.out.println("[VisionSnap] " + message);
        DriverStation.reportWarning(message, false);
    }

    /**
     * Logs synchronization failure to the Driver Station.
     */
    private void reportFailure(String reason) {
        String message = "SNAP FAILED: " + reason;
        System.out.println("[VisionSnap] " + message);
        DriverStation.reportWarning(message, false);
    }
}
