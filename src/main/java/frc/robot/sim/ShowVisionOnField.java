package frc.robot.sim;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.visutils.ShowIcon;

import java.util.List;
import java.util.Optional;

/**
 * Class to show vision targets on the field.
 */
public class ShowVisionOnField {
    /** Module locations relative to robot center (from TunerConstants). */
    private final Translation2d[] MODULE_LOCATIONS;

    private final Optional<Field2d> m_realGlassField;
    private final Optional<Field2d> m_simulationField;

    /** Icon for displaying the Kalman-filtered vision pose (converged vs not). */
    private final ShowIcon m_kalmanIcon = new ShowIcon(
        List.of("ZKalmanVisionPoseConverged", "ZKalmanVisionPoseNotConverged"));

    /** Icon for displaying the blue tape location. */
    private final ShowIcon m_blueTape = new ShowIcon(List.of("AAABlueTape"));

    /** Icon for displaying the red tape location. */
    private final ShowIcon m_redTape = new ShowIcon(List.of("AAARedTape"));

    /**
     * Creates a new ShowVisionOnField.
     *
     * @param realGlassField The field displaying real/estimated robot pose (may be null)
     * @param simulationField The field displaying simulated/ground truth pose (may be null)
     */
    public ShowVisionOnField(Field2d realGlassField, Field2d simulationField, BotConfigInterface swerveConstants) {
        MODULE_LOCATIONS = new Translation2d[4];
        MODULE_LOCATIONS[0] = new Translation2d(swerveConstants.getFrontLeft().LocationX, swerveConstants.getFrontLeft().LocationY);
        MODULE_LOCATIONS[1] = new Translation2d(swerveConstants.getFrontRight().LocationX, swerveConstants.getFrontRight().LocationY);
        MODULE_LOCATIONS[2] = new Translation2d(swerveConstants.getBackLeft().LocationX, swerveConstants.getBackLeft().LocationY);
        MODULE_LOCATIONS[3] = new Translation2d(swerveConstants.getBackRight().LocationX, swerveConstants.getBackRight().LocationY);

        if (realGlassField == null) {
            throw new IllegalArgumentException("realField cannot be null");
        }

        // Only when we're debugging in simulation do we show extra visualizations on the
        // PhotonVision sim field.
        if (simulationField != null && !Robot.isSimulation()) {
            throw new IllegalArgumentException("simulationField should only be in isSimulation mode");
        }
        if (simulationField == null && Robot.isSimulation()) {
            throw new IllegalArgumentException("simulationField cannot be null in simulation mode");
        }

        m_realGlassField = Optional.ofNullable(realGlassField);
        m_simulationField = Optional.ofNullable(simulationField);
    }

    /**
     * Shows the estimated robot pose and wheel positions on the field.
     *
     * @param driveState The current swerve drive state containing pose and module states
     */
    public void showEstimatedPoseAndWheels(
        SwerveDrivetrain.SwerveDriveState driveState) {

        // Always show robot pose and wheels on real/glass field
        m_realGlassField.ifPresent(f -> {
            f.getObject("EstimatedRobot").setPose(driveState.Pose);
            f.getObject("EstimatedRobotModules").setPoses(getModulePoses(driveState));
        });

        // Only show on debug field if simulation is running in debug mode
        if (Robot.isSimulation()) {
            m_simulationField.ifPresent(f -> {
                f.getObject("EstimatedRobot").setPose(driveState.Pose);
                f.getObject("EstimatedRobotModules").setPoses(getModulePoses(driveState));
            });
        }
    }

    /**
     * Shows the ground truth robot pose on the field.
     *
     * @param groundTruthPose The ground truth pose (where the robot actually is in simulation)
     */
    public void showGroundTruthPoseOnField(Pose2d groundTruthPose) {
        // Only show ground truth pose on glass field when we're in simulation mode
        if (Robot.isSimulation()) {
            m_realGlassField.ifPresent(f -> f.getObject("GroundTruthRobot").setPose(groundTruthPose));

            // Also, the default "Robot" object on the glass field shows same thing.  This is confusing,
            // but PhotonVisions sim is updating Robot based on where the cameras are.  And then I'm
            // adding another object called GroundTruthRobot that shows the same pose.
            m_realGlassField.ifPresent(f -> f.getObject("Robot").setPose(groundTruthPose));
        }

        // Only show on debug field if simulation is running in debug mode
        if (Robot.isSimulation()) {
            m_simulationField.ifPresent(f -> f.getObject("GroundTruthRobot").setPose(groundTruthPose));
        }
    }

    /**
     * Shows or hides the point-in-time vision estimate on the field.
     *
     * @param visionPose The vision pose if present, or empty to hide the estimate
     */
    public void showPointInTimeVisionEstimate(Optional<Pose2d> visionPose) {
        // Always show point in time vision pose on real/glass field
        m_realGlassField.ifPresent(f -> {
            visionPose.ifPresentOrElse(
                pose -> f.getObject("VisionEstimation").setPose(pose),
                () -> f.getObject("VisionEstimation").setPoses()
            );
        });

        // Only show on debug field if simulation is running in debug mode
        if (Robot.isSimulation()) {
            m_simulationField.ifPresent(f -> {
                visionPose.ifPresentOrElse(
                    pose -> f.getObject("VisionEstimation").setPose(pose),
                    () -> f.getObject("VisionEstimation").setPoses()
                );
            });
        }
    }

    /**
     * Shows or hides the Kalman-filtered vision pose on the field.
     * Uses different field objects based on showIndex for different appearances
     * (e.g. converged=0, not converged=1).
     *
     * @param kalmanPose The Kalman-filtered pose if present, or empty to hide
     * @param showIndex Index selecting which appearance to use (0=converged, 1=not converged)
     */
    public void showKalmanVisionPose(Optional<Pose2d> kalmanPose, int showIndex) {
        showIcon(m_kalmanIcon, kalmanPose, showIndex);
    }

    /**
     * Shows or hides the blue tape location on the field.
     *
     * @param pose The pose to display, or empty to hide
     */
    public void showBlueTape(Optional<Pose2d> pose) {
        showIcon(m_blueTape, pose, 0);
    }

    /**
     * Shows or hides the red tape location on the field.
     *
     * @param pose The pose to display, or empty to hide
     */
    public void showRedTape(Optional<Pose2d> pose) {
        showIcon(m_redTape, pose, 0);
    }

    /**
     * Shows or hides an icon on both fields (real and simulation).
     * Delegates to ShowIcon.show() for each field.
     *
     * @param icon The ShowIcon instance defining which field objects to use
     * @param pose The pose to display, or empty to hide all objects
     * @param showIndex Index into the icon's object names selecting which to display
     */
    public void showIcon(ShowIcon icon, Optional<Pose2d> pose, int showIndex) {
        // Always show on real/glass field
        icon.show(m_realGlassField, pose, showIndex);

        // Only show on debug field if simulation is running in debug mode
        if (Robot.isSimulation()) {
            icon.show(m_simulationField, pose, showIndex);
        }
    }

    /**
     * Get the Pose2d of each swerve module based on the current robot pose and module states.
     */
    private Pose2d[] getModulePoses(SwerveDrivetrain.SwerveDriveState driveState) {
        Pose2d[] modulePoses = new Pose2d[4];
        for (int i = 0; i < 4; i++) {
            modulePoses[i] = driveState.Pose.transformBy(
                new Transform2d(MODULE_LOCATIONS[i], driveState.ModuleStates[i].angle)
            );
        }
        return modulePoses;
    }
}
