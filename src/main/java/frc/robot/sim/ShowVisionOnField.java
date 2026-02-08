package frc.robot.sim;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import java.util.List;
import java.util.Optional;

/**
 * Class to show vision targets on the field.
 */
public class ShowVisionOnField {
    /** Module locations relative to robot center (from TunerConstants). */
    private static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };

    private final Optional<Field2d> m_realGlassField;
    private final Optional<Field2d> m_simulationField;

    /**
     * Creates a new ShowVisionOnField.
     *
     * @param realGlassField The field displaying real/estimated robot pose (may be null)
     * @param simulationField The field displaying simulated/ground truth pose (may be null)
     */
    public ShowVisionOnField(Field2d realGlassField, Field2d simulationField) {
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

    private void showIconHelper(
        Optional<Field2d> field,
        Optional<Pose2d> pose,
        List<String> objectNames,
        int showIndex) {

        field.ifPresent(f -> {
            // Clear all objects first
            for (String name : objectNames) {
                f.getObject(name).setPoses();
            }

            // Show on the selected object
            pose.ifPresent(p -> f.getObject(objectNames.get(showIndex)).setPose(p));
        });
    }

    /**
     * Shows or hides an icon on the field, selecting one of several named objects.
     * All objects are cleared first, then the pose is set on the object at showIndex.
     * This allows different field object appearances (e.g. color) based on state.
     *
     * @param pose The pose to display, or empty to hide all objects
     * @param objectNames The list of field object names (each can have a different appearance)
     * @param showIndex Index into objectNames selecting which object to display
     */
    public void showIcon(Optional<Pose2d> pose, List<String> objectNames, int showIndex) {
        // Always show on real/glass field
        showIconHelper(m_realGlassField, pose, objectNames, showIndex);

        // Only show on debug field if simulation is running in debug mode
        if (Robot.isSimulation()) {
            showIconHelper(m_simulationField, pose, objectNames, showIndex);
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
