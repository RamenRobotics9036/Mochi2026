// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/**
 * VisionSubsystem manages a dual-Limelight setup with pipeline switching:
 * <ul>
 *   <li><b>Fixed Camera:</b> Chassis-mounted for MegaTag2 field localization</li>
 *   <li><b>Turret Camera:</b> Hood-mounted for target tracking (AprilTags OR Fuel)</li>
 * </ul>
 * 
 * <p>The turret camera supports two modes:
 * <ul>
 *   <li>{@link VisionMode#APRILTAG}: Pipeline 0 - AprilTag detection</li>
 *   <li>{@link VisionMode#FUEL}: Pipeline 1 - Color/GRIP fuel (ball) detection</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

    /**
     * Vision modes for the turret camera.
     */
    public enum VisionMode {
        /** AprilTag detection mode (Pipeline 0) */
        APRILTAG,
        /** Fuel (ball) detection mode (Pipeline 1) */
        FUEL
    }

    private final CommandSwerveDrivetrain m_drivetrain;
    private VisionMode m_currentMode = VisionMode.APRILTAG;

    // ========== FIXED CAMERA (Localization) State ==========
    private int m_fixedTagCount = 0;
    private double m_fixedAvgTagDist = 0.0;
    private boolean m_hasValidPose = false;

    // ========== TURRET CAMERA (Targeting) State ==========
    private double m_turretTX = 0.0;
    private double m_turretTY = 0.0;
    private boolean m_hasTurretTarget = false;

    // ========== FUEL DETECTION State ==========
    private double m_fuelTX = 0.0;
    private double m_fuelTY = 0.0;
    private boolean m_hasFuelTarget = false;

    /**
     * Creates a new VisionSubsystem managing both Limelights.
     * 
     * @param drivetrain The swerve drivetrain for pose updates and gyro access
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        initShuffleboard();
        
        // Start in AprilTag mode
        setTurretMode(VisionMode.APRILTAG);
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

        // Turret Camera telemetry
        tab.addString("Turret Mode", () -> m_currentMode.name());
        tab.addNumber("Turret: TX", () -> m_turretTX);
        tab.addBoolean("Turret: Has Target", () -> m_hasTurretTarget);
        
        // Fuel detection telemetry
        tab.addNumber("Fuel: TX", () -> m_fuelTX);
        tab.addBoolean("Fuel: Has Target", () -> m_hasFuelTarget);
        
        // Best aiming target
        tab.addNumber("Aiming TX", this::getAimingTX);

        // Add Camera Streams (Actual Video)
        // These will appear as "Camera Stream" widgets in Shuffleboard
        tab.addCamera("Fixed Stream", "limelight-fixed", "http://limelight-fixed.local:5800/stream.mjpg")
           .withWidget(BuiltInWidgets.kCameraStream)
           .withSize(3, 3)
           .withPosition(0, 3);

        tab.addCamera("Turret Stream", "limelight-turret", "http://limelight-turret.local:5800/stream.mjpg")
           .withWidget(BuiltInWidgets.kCameraStream)
           .withSize(3, 3)
           .withPosition(3, 3);
    }

    @Override
    public void periodic() {
        // ==================== TASK A: Fixed Camera (MegaTag2 Localization) ====================
        updateFixedCameraLocalization();

        // ==================== TASK B: Turret Camera (Mode-Based Tracking) ====================
        updateTurretCameraTracking();
    }

    /**
     * Task A: Process the fixed (chassis-mounted) camera for MegaTag2 localization.
     */
    private void updateFixedCameraLocalization() {
        // Step 1: Get robot orientation from Pigeon2 IMU
        Rotation3d robotRotation = m_drivetrain.getPigeon2().getRotation3d();
        
        // Step 2: Sync robot orientation with fixed Limelight for MegaTag2
        LimelightHelpers.SetRobotOrientation(
            VisionConstants.kFixedCameraName,
            Units.radiansToDegrees(robotRotation.getZ()),  // Yaw
            0.0,  // Yaw rate
            Units.radiansToDegrees(robotRotation.getY()),  // Pitch
            0.0,  // Pitch rate
            Units.radiansToDegrees(robotRotation.getX()),  // Roll
            0.0   // Roll rate
        );

        // Step 3: Get MegaTag2 pose estimate
        PoseEstimate poseEstimate = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kFixedCameraName);

        // Step 4: Validate and update pose estimator
        if (poseEstimate != null) {
            m_fixedTagCount = poseEstimate.tagCount;
            m_fixedAvgTagDist = poseEstimate.avgTagDist;

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
     * Task B: Read targeting data from the turret camera based on current mode.
     */
    private void updateTurretCameraTracking() {
        // Always read turret camera data (works for both AprilTag and Fuel modes)
        m_hasTurretTarget = LimelightHelpers.getTV(VisionConstants.kTurretCameraName);

        if (m_hasTurretTarget) {
            m_turretTX = LimelightHelpers.getTX(VisionConstants.kTurretCameraName);
            m_turretTY = LimelightHelpers.getTY(VisionConstants.kTurretCameraName);
        } else {
            m_turretTX = 0.0;
            m_turretTY = 0.0;
        }

        // Update fuel-specific state based on current mode
        if (m_currentMode == VisionMode.FUEL) {
            // In fuel mode, check if target passes the TY filter
            boolean passesFilter = m_turretTY > VisionConstants.FUEL_TY_FILTER;
            m_hasFuelTarget = m_hasTurretTarget && passesFilter;
            
            if (m_hasFuelTarget) {
                m_fuelTX = m_turretTX;
                m_fuelTY = m_turretTY;
            } else {
                m_fuelTX = 0.0;
                m_fuelTY = 0.0;
            }
        } else {
            // Not in fuel mode - no fuel target
            m_hasFuelTarget = false;
            m_fuelTX = 0.0;
            m_fuelTY = 0.0;
        }
    }

    // ==================== PIPELINE SWITCHING ====================

    /**
     * Sets the turret camera mode (switches the Limelight pipeline).
     * 
     * @param mode The desired vision mode (APRILTAG or FUEL)
     */
    public void setTurretMode(VisionMode mode) {
        m_currentMode = mode;
        
        int pipelineIndex = switch (mode) {
            case APRILTAG -> VisionConstants.PIPELINE_TAGS;
            case FUEL -> VisionConstants.PIPELINE_FUEL;
        };
        
        LimelightHelpers.setPipelineIndex(VisionConstants.kTurretCameraName, pipelineIndex);
    }

    /**
     * @return The current turret camera vision mode
     */
    public VisionMode getTurretMode() {
        return m_currentMode;
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

    // ==================== TURRET CAMERA GETTERS (AprilTag Mode) ====================

    /**
     * Gets the horizontal offset to the target from the turret camera.
     * 
     * @return Horizontal offset (tx) in degrees. Positive = target is to the right.
     */
    public double getTurretX() {
        return m_turretTX;
    }

    /**
     * Gets the vertical offset to the target from the turret camera.
     * 
     * @return Vertical offset (ty) in degrees.
     */
    public double getTurretY() {
        return m_turretTY;
    }

    /**
     * @return True if the turret camera has locked onto a target
     */
    public boolean hasTurretTarget() {
        return m_hasTurretTarget;
    }

    // ==================== FUEL GETTERS ====================

    /**
     * Gets the horizontal offset to the fuel (ball) target.
     * Only valid when mode is set to FUEL.
     * 
     * @return Horizontal offset (tx) in degrees. Positive = fuel is to the right.
     */
    public double getFuelTX() {
        return m_fuelTX;
    }

    /**
     * Gets the vertical offset to the fuel target.
     * 
     * @return Vertical offset (ty) in degrees.
     */
    public double getFuelTY() {
        return m_fuelTY;
    }

    /**
     * Checks if the turret camera currently sees a valid fuel target.
     * Returns true ONLY if:
     * <ul>
     *   <li>Pipeline is set to FUEL mode</li>
     *   <li>tv == 1 (target detected)</li>
     *   <li>ty passes the FUEL_TY_FILTER (not too far)</li>
     * </ul>
     * 
     * @return True if a valid fuel target is detected
     */
    public boolean hasFuelTarget() {
        return m_hasFuelTarget;
    }

    // ==================== BEST TARGET SELECTOR ====================

    /**
     * Automatically returns the best TX for aiming based on current mode.
     * <ul>
     *   <li>APRILTAG mode: Returns turret TX</li>
     *   <li>FUEL mode: Returns fuel TX</li>
     * </ul>
     * 
     * Use this method in your turret/aiming code for mode-agnostic control.
     * 
     * @return Horizontal offset (tx) in degrees for the current targeting mode
     */
    public double getAimingTX() {
        return switch (m_currentMode) {
            case APRILTAG -> m_turretTX;
            case FUEL -> m_fuelTX;
        };
    }

    /**
     * @return True if there is a valid target for the current mode
     */
    public boolean hasAimingTarget() {
        return switch (m_currentMode) {
            case APRILTAG -> m_hasTurretTarget;
            case FUEL -> m_hasFuelTarget;
        };
    }
}
