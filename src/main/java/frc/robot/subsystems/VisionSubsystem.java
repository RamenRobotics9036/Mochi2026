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
 * Subsystem for managing dual Limelight cameras with multi-pipeline processing.
 * 
 * <p>Integrates a fixed camera for chassis localization using MegaTag2 and a turret-mounted 
 * camera for active target tracking (AprilTags or Fuel/Game Pieces).
 */
public class VisionSubsystem extends SubsystemBase {

    /** Vision modes representing different targeting pipelines for the turret camera. */
    public enum VisionMode {
        /** AprilTag detection mode (Pipeline 0). */
        APRILTAG,
        /** Fuel (ball) detection mode (Pipeline 1). */
        FUEL
    }

    /** Reference to the drivetrain subsystem for pose updates. */
    private final CommandSwerveDrivetrain m_drivetrain;
    private VisionMode m_currentMode = VisionMode.APRILTAG;

    /** Total number of AprilTags currently visible to the localization camera. */
    private int m_fixedTagCount = 0;
    /** The average distance in meters from the fixed camera to all visible AprilTags. */
    private double m_fixedAvgTagDist = 0.0;
    /** True if the localization camera meets distance and tag count safety thresholds. */
    private boolean m_hasValidPose = false;

    /** Horizontal angular offset in degrees from the turret camera crosshair to the target. */
    private double m_turretTX = 0.0;
    /** Vertical angular offset in degrees from the turret camera crosshair to the target. */
    private double m_turretTY = 0.0;
    /** True if the turret camera currently sees any valid target in its current pipeline. */
    private boolean m_hasTurretTarget = false;

    /** Horizontal angular offset in degrees specifically for fuel/game piece targets. */
    private double m_fuelTX = 0.0;
    /** Vertical angular offset in degrees specifically for fuel/game piece targets. */
    private double m_fuelTY = 0.0;
    /** True if a fuel target is found and passes the vertical (TY) proximity filter. */
    private boolean m_hasFuelTarget = false;

    /**
     * Creates a new VisionSubsystem.
     * 
     * @param drivetrain The drivetrain instance to receive vision-based pose updates.
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        // Store drivetrain reference
        m_drivetrain = drivetrain;
        // Initialize Shuffleboard telemetry
        initShuffleboard();
        
        // Ensure Limelight hardware starts in a known pipeline
        setTurretMode(VisionMode.APRILTAG);
    }

    /** Sets up real-time telemetry and camera streams for driver and pit visualization. */
    private void initShuffleboard() {
        // Create or get the "Vision" tab on Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        
        // Fixed camera localization data
        tab.addNumber("Fixed: Tag Count", () -> m_fixedTagCount);
        tab.addNumber("Fixed: Avg Tag Dist (m)", () -> m_fixedAvgTagDist);
        tab.addBoolean("Fixed: Valid Pose", () -> m_hasValidPose);

        // Turret camera targeting data
        tab.addString("Turret Mode", () -> m_currentMode.name());
        tab.addNumber("Turret: TX", () -> m_turretTX);
        tab.addBoolean("Turret: Has Target", () -> m_hasTurretTarget);
        
        // Fuel-specific targeting data
        tab.addNumber("Fuel: TX", () -> m_fuelTX);
        tab.addBoolean("Fuel: Has Target", () -> m_hasFuelTarget);

        // Best aiming target
        tab.addNumber("Aiming TX", this::getAimingTX);

        // Add Camera Streams (Actual Video)
        // These will appear as "Camera Stream" widgets in Shuffleboard
        tab.addCamera("Fixed Stream", "limelight-fixed", "http://limelight-fixed.local")
           .withWidget(BuiltInWidgets.kCameraStream)
           .withSize(3, 3)
           .withPosition(0, 3);

        tab.addCamera("Turret Stream", "limelight-turret", "http://limelight-turret.local")
           .withWidget(BuiltInWidgets.kCameraStream)
           .withSize(3, 3)
           .withPosition(3, 3);
    }

    /**
     * Updates localization data and tracking state based on high-frequency Limelight networking.
     */
    @Override
    public void periodic() {
        // Update fixed camera pose estimation and drivetrain localization
        updateFixedCameraLocalization();

        // Update turret camera target tracking data
        updateTurretCameraTracking();
    }

    /**
     * Syncs drivetrain orientation with Limelight and applies vision pose measurements.
     * Uses MegaTag2 for robust localization during aggressive robot motion.
     */
    private void updateFixedCameraLocalization() {
        // Sync robot orientation from drivetrain gyro to Limelight
        Rotation3d robotRotation = m_drivetrain.getPigeon2().getRotation3d();
        
        // Update Limelight with current robot orientation (degrees)
        LimelightHelpers.SetRobotOrientation(
            VisionConstants.kFixedCameraName,
            Units.radiansToDegrees(robotRotation.getZ()),
            0.0,
            Units.radiansToDegrees(robotRotation.getY()),
            0.0,
            Units.radiansToDegrees(robotRotation.getX()),
            0.0
        );

        // Retrieve pose estimate using MegaTag2 method
        PoseEstimate poseEstimate = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kFixedCameraName);

        // Update internal state based on pose estimate validity
        if (poseEstimate != null) {
            m_fixedTagCount = poseEstimate.tagCount;
            m_fixedAvgTagDist = poseEstimate.avgTagDist;

            // Validate pose estimate against tag count and distance thresholds
            boolean isValid = poseEstimate.tagCount >= VisionConstants.MIN_TAG_COUNT
                           && poseEstimate.avgTagDist < VisionConstants.MAX_TAG_DISTANCE;

            // Store validity state
            m_hasValidPose = isValid;

            // If valid, add the vision measurement to the drivetrain's pose estimator
            if (isValid) {
                m_drivetrain.addVisionMeasurement(
                    poseEstimate.pose,
                    poseEstimate.timestampSeconds
                );
            }
        // else no pose estimate available
        } else {
            m_fixedTagCount = 0;
            m_fixedAvgTagDist = 0.0;
            m_hasValidPose = false;
        }
    }

    /** Reads and filters horizontal (TX) and vertical (TY) targeting data from the turret camera. */
    private void updateTurretCameraTracking() {
        // Read target validity and offsets from Limelight
        m_hasTurretTarget = LimelightHelpers.getTV(VisionConstants.kTurretCameraName);

        // Get raw TX and TY if target is present
        if (m_hasTurretTarget) {
            m_turretTX = LimelightHelpers.getTX(VisionConstants.kTurretCameraName);
            m_turretTY = LimelightHelpers.getTY(VisionConstants.kTurretCameraName);
        // else no target
        } else {
            m_turretTX = 0.0;
            m_turretTY = 0.0;
        }

        // Apply fuel target filtering if in FUEL mode
        if (m_currentMode == VisionMode.FUEL) {
            // Determine if target passes vertical proximity filter
            boolean passesFilter = m_turretTY > VisionConstants.FUEL_TY_FILTER;
            m_hasFuelTarget = m_hasTurretTarget && passesFilter;
            
            // Set fuel TX/TY based on filtered target presence
            if (m_hasFuelTarget) {
                m_fuelTX = m_turretTX;
                m_fuelTY = m_turretTY;
            // else no valid fuel target
            } else {
                m_fuelTX = 0.0;
                m_fuelTY = 0.0;
            }
        // else not in FUEL mode
        } else {
            // ensure fuel target state is cleared
            m_hasFuelTarget = false;
            m_fuelTX = 0.0;
            m_fuelTY = 0.0;
        }
    }
    //  End of periodic

    /**
     * Sets the turret camera mode and sends a command to switch Limelight hardware pipelines.
     * 
     * @param mode The desired vision mode (e.g., APRILTAG or FUEL).
     */
    public void setTurretMode(VisionMode mode) {
        m_currentMode = mode;
        
        // Determine pipeline index based on selected mode
        int pipelineIndex = switch (mode) {
            case APRILTAG -> VisionConstants.PIPELINE_TAGS;
            case FUEL -> VisionConstants.PIPELINE_FUEL;
        };
        
        // Command Limelight to switch to the appropriate pipeline
        LimelightHelpers.setPipelineIndex(VisionConstants.kTurretCameraName, pipelineIndex);
    }

    /** @return The current vision mode for the turret camera. */
    public VisionMode getTurretMode() {
        // currently selected vision mode
        return m_currentMode;
    }

    // fixing camera getters
    /**
     * @return True if the fixed camera pose is currently being used for localization
     */
    public boolean hasValidPose() {
        // valid pose state from fixed camera
        return m_hasValidPose;
    }

    /**
     * @return The number of AprilTags currently visible to the fixed camera
     */
    public int getFixedTagCount() {
        // fixed camera tag count
        return m_fixedTagCount;
    }

    /**
     * @return Average distance to visible tags (meters) from fixed camera
     */
    public double getFixedAvgTagDistance() {
        // fixed camera average tag distance
        return m_fixedAvgTagDist;
    }

    // turret camera getters (AprilTag Mode)

    /**
     * Gets the horizontal offset to the target from the turret camera.
     * 
     * @return Horizontal offset (tx) in degrees. Positive = target is to the right.
     */
    public double getTurretX() {
        // horizontal offset from turret camera
        return m_turretTX;
    }

    /**
     * Gets the vertical offset to the target from the turret camera.
     * 
     * @return Vertical offset (ty) in degrees.
     */
    public double getTurretY() {
        // vertical offset from turret camera
        return m_turretTY;
    }

    /**
     * @return True if the turret camera has locked onto a target
     */
    public boolean hasTurretTarget() {
        // turret target presence
        return m_hasTurretTarget;
    }

    // Getting fuel getters

    /**
     * Gets the horizontal offset to the fuel (ball) target.
     * Only valid when mode is set to FUEL.
     * 
     * @return Horizontal offset (tx) in degrees. Positive = fuel is to the right.
     */
    public double getFuelTX() {
        // horizontal offset from fuel target
        return m_fuelTX;
    }

    /**
     * Gets the vertical offset to the fuel target.
     * 
     * @return Vertical offset (ty) in degrees.
     */
    public double getFuelTY() {
        // vertical offset from fuel target
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
        // fuel target presence
        return m_hasFuelTarget;
    }

    // selecting best target

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
        // best horizontal offset based on mode
        return switch (m_currentMode) {
            case APRILTAG -> m_turretTX;
            case FUEL -> m_fuelTX;
        };
    }

    /**
     * @return True if there is a valid target for the current mode
     */
    public boolean hasAimingTarget() {
        // best target presence based on mode
        return switch (m_currentMode) {
            case APRILTAG -> m_hasTurretTarget;
            case FUEL -> m_hasFuelTarget;
        };
    }
}
