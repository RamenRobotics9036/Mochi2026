// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * VisionSubsystem manages a dual-Limelight setup:
 * <ul>
 *   <li><b>Fixed Camera:</b> Chassis-mounted for MegaTag2 field localization</li>
 *   <li><b>Turret Camera:</b> Hood-mounted for target tracking (tx/ty)</li>
 * </ul>
 * 
 * <p>MegaTag2 requires the robot's orientation (from the Pigeon2 IMU) to be
 * synced with the fixed Limelight before pose estimation can work correctly.
 */
public class VisionSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain m_drivetrain;

    // ========== FIXED CAMERA (Localization) State ==========
    private int m_fixedTagCount = 0;
    private double m_fixedAvgTagDist = 0.0;
    private boolean m_hasValidPose = false;

    // ========== TURRET CAMERA (Targeting) State ==========
    private double m_turretTX = 0.0;
    private double m_turretTY = 0.0;
    private boolean m_hasTurretTarget = false;

    /**
     * Creates a new VisionSubsystem managing both Limelights.
     * 
     * @param drivetrain The swerve drivetrain for pose updates and gyro access
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        initShuffleboard();
    }

    /**
     * Initialize Shuffleboard telemetry for both cameras.
     */
    private void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        
        // Fixed Camera (Localization) telemetry
        tab.addNumber("Fixed: Tag Count", () -> m_fixedTagCount);
        tab.addNumber("Fixed: Avg Tag Dist (m)", () -> m_fixedAvgTagDist);
        tab.addBoolean("Fixed: Valid Pose", () -> m_hasValidPose);
        tab.addBoolean("Fixed: Has Tags", () -> 
            LimelightHelpers.getTV(VisionConstants.kFixedCameraName));

        // Turret Camera (Targeting) telemetry
        tab.addNumber("Turret: TX", () -> m_turretTX);
        tab.addNumber("Turret: TY", () -> m_turretTY);
        tab.addBoolean("Turret: Has Target", () -> m_hasTurretTarget);
    }

    @Override
    public void periodic() {
        // ==================== TASK A: Fixed Camera (MegaTag2 Localization) ====================
        updateFixedCameraLocalization();

        // ==================== TASK B: Turret Camera (Target Tracking) ====================
        updateTurretCameraTracking();
    }

    /**
     * Task A: Process the fixed (chassis-mounted) camera for MegaTag2 localization.
     * <ol>
     *   <li>Get Pigeon2 rotation</li>
     *   <li>Sync orientation with Limelight</li>
     *   <li>Get MegaTag2 pose estimate</li>
     *   <li>Filter and update drivetrain pose estimator</li>
     * </ol>
     */
    private void updateFixedCameraLocalization() {
        // Step 1: Get robot orientation from Pigeon2 IMU
        Rotation3d robotRotation = m_drivetrain.getPigeon2().getRotation3d();
        
        // Step 2: Sync robot orientation with fixed Limelight for MegaTag2
        LimelightHelpers.SetRobotOrientation(
            VisionConstants.kFixedCameraName,
            Units.radiansToDegrees(robotRotation.getZ()),  // Yaw
            0.0,  // Yaw rate (not needed)
            Units.radiansToDegrees(robotRotation.getY()),  // Pitch
            0.0,  // Pitch rate
            Units.radiansToDegrees(robotRotation.getX()),  // Roll
            0.0   // Roll rate
        );

        // Step 3: Get MegaTag2 pose estimate (WPILib Blue alliance origin)
        PoseEstimate poseEstimate = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kFixedCameraName);

        // Step 4: Validate and update pose estimator
        if (poseEstimate != null) {
            m_fixedTagCount = poseEstimate.tagCount;
            m_fixedAvgTagDist = poseEstimate.avgTagDist;

            // Filter: Only accept poses with enough tags and within distance limit
            boolean isValid = poseEstimate.tagCount >= VisionConstants.MIN_TAG_COUNT
                           && poseEstimate.avgTagDist < VisionConstants.MAX_TAG_DISTANCE;

            m_hasValidPose = isValid;

            if (isValid) {
                m_drivetrain.addVisionMeasurement(
                    poseEstimate.pose,
                    poseEstimate.timestampSeconds
                );
            }
        } else {
            m_fixedTagCount = 0;
            m_fixedAvgTagDist = 0.0;
            m_hasValidPose = false;
        }
    }

    /**
     * Task B: Read targeting data from the turret (hood-mounted) camera.
     * Simply reads tx, ty, and tv for use by the Turret subsystem.
     */
    private void updateTurretCameraTracking() {
        // Read whether turret camera has a valid target
        m_hasTurretTarget = LimelightHelpers.getTV(VisionConstants.kTurretCameraName);

        if (m_hasTurretTarget) {
            // Read horizontal and vertical offsets to target
            m_turretTX = LimelightHelpers.getTX(VisionConstants.kTurretCameraName);
            m_turretTY = LimelightHelpers.getTY(VisionConstants.kTurretCameraName);
        } else {
            // No target - reset values
            m_turretTX = 0.0;
            m_turretTY = 0.0;
        }
    }

    // ==================== FIXED CAMERA GETTERS ====================

    /**
     * @return True if the fixed camera pose is currently being used for localization
     */
    public boolean hasValidPose() {
        return m_hasValidPose;
    }

    /**
     * @return The number of AprilTags currently visible to the fixed camera
     */
    public int getFixedTagCount() {
        return m_fixedTagCount;
    }

    /**
     * @return Average distance to visible tags (meters) from fixed camera
     */
    public double getFixedAvgTagDistance() {
        return m_fixedAvgTagDist;
    }

    // ==================== TURRET CAMERA GETTERS ====================

    /**
     * Gets the horizontal offset to the target from the turret camera.
     * Use this value to control turret aiming.
     * 
     * @return Horizontal offset (tx) in degrees. Positive = target is to the right.
     */
    public double getTurretX() {
        return m_turretTX;
    }

    /**
     * Gets the vertical offset to the target from the turret camera.
     * Use this value for distance estimation or shooter angle.
     * 
     * @return Vertical offset (ty) in degrees. Positive = target is above crosshair.
     */
    public double getTurretY() {
        return m_turretTY;
    }

    /**
     * Checks if the turret camera currently sees a valid target.
     * 
     * @return True if the turret camera has locked onto a target
     */
    public boolean hasTurretTarget() {
        return m_hasTurretTarget;
    }
}
