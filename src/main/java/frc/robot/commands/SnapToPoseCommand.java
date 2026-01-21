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
 * SnapToPoseCommand: Instantly resets the drivetrain odometry to match the vision pose.
 * 
 * <p><b>Team 2910 Strategy:</b> Before critical actions (like shooting), the driver
 * holds a button to "snap" the robot's internal pose to the vision-measured pose.
 * This eliminates accumulated odometry drift from aggressive driving (spinning, collisions).
 * 
 * <p><b>When to use:</b>
 * <ul>
 *   <li>Before shooting - ensures aiming calculations use accurate pose</li>
 *   <li>After heavy defense - resets drift from collisions</li>
 *   <li>After spinning - resets gyro-related drift</li>
 * </ul>
 * 
 * <p><b>Safety:</b> This command only resets if the vision data is reliable:
 * <ul>
 *   <li>At least one AprilTag visible</li>
 *   <li>Average tag distance under 3.5 meters</li>
 * </ul>
 * 
 * <p>This is an instant command - it executes once and finishes immediately.
 */
public class SnapToPoseCommand extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private boolean m_snapSucceeded = false;

    /**
     * Creates a new SnapToPoseCommand.
     * 
     * @param drivetrain The swerve drivetrain to reset
     */
    public SnapToPoseCommand(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        
        // No subsystem requirements - this is a one-shot instant command
        // We don't want to interrupt driving while snapping
    }

    @Override
    public void initialize() {
        m_snapSucceeded = false;
        
        try {
            // Get MegaTag2 pose from fixed camera
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                VisionConstants.kFixedCameraName
            );

            // Safety check: validate the pose estimate
            if (poseEstimate == null) {
                reportFailure("No pose estimate available");
                return;
            }

            if (poseEstimate.tagCount < VisionConstants.MIN_TAG_COUNT) {
                reportFailure("Not enough tags visible (need " + VisionConstants.MIN_TAG_COUNT 
                    + ", have " + poseEstimate.tagCount + ")");
                return;
            }

            if (poseEstimate.avgTagDist > VisionConstants.SNAP_MAX_DISTANCE) {
                reportFailure("Tags too far (max " + VisionConstants.SNAP_MAX_DISTANCE 
                    + "m, current " + String.format("%.2f", poseEstimate.avgTagDist) + "m)");
                return;
            }

            // All checks passed - snap the odometry!
            Pose2d visionPose = poseEstimate.pose;
            m_drivetrain.resetPose(visionPose);
            
            m_snapSucceeded = true;
            reportSuccess(visionPose, poseEstimate.tagCount, poseEstimate.avgTagDist);

        } catch (Exception e) {
            // Thread safety: don't crash if camera disconnected
            reportFailure("Exception: " + e.getMessage());
        }
    }

    @Override
    public boolean isFinished() {
        // Instant command - always finish immediately
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        // Allow snapping while disabled for testing
        return true;
    }

    /**
     * @return True if the last snap attempt succeeded
     */
    public boolean snapSucceeded() {
        return m_snapSucceeded;
    }

    /**
     * Reports a successful snap to the console and dashboard.
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
     * Reports a failed snap attempt to the console and dashboard.
     */
    private void reportFailure(String reason) {
        String message = "SNAP FAILED: " + reason;
        System.out.println("[VisionSnap] " + message);
        DriverStation.reportWarning(message, false);
    }
}
