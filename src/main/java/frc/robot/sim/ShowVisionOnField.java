package frc.robot.sim;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.SimConstants.SimMode;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import java.util.Optional;

/**
 * Class to show vision targets on the field.
 */
public class ShowVisionOnField {
    // $TODO - This should go away
    public enum FieldType {
        REAL_GLASS_FIELD,
        SIMULATION_FIELD
    }

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
     * @param fieldType The field to display on (REAL_GLASS_FIELD or SIMULATION_FIELD)
     * @param groundTruthPose The ground truth pose (where the robot actually is in simulation)
     */
    public void showGroundTruthPoseOnField(FieldType fieldType, Pose2d groundTruthPose) {
        // $TODO
        return;

        // if (fieldType == FieldType.SIMULATION_FIELD && !SimCheck.isSimulationDebug()) {
        //     throw new IllegalArgumentException("We dont draw on sim field unless SIM_DEBUG mode");
        // }

        // Optional<Field2d> field = (fieldType == FieldType.REAL_GLASS_FIELD) ? m_realGlassField : m_simulationField;
        // field.ifPresent(f -> f.getObject("GroundTruthRobot").setPose(groundTruthPose));
    }

    /**
     * Shows or hides the point-in-time vision estimate on the field.
     *
     * @param fieldType The field to display on (REAL_GLASS_FIELD or SIMULATION_FIELD)
     * @param visionPose The vision pose if present, or empty to hide the estimate
     */
    public void showPointInTimeVisionEstimate(FieldType fieldType, Optional<Pose2d> visionPose) {
        // $TODO
        return;

        // if (fieldType == FieldType.SIMULATION_FIELD && !SimCheck.isSimulationDebug()) {
        //     throw new IllegalArgumentException("We dont draw on sim field unless SIM_DEBUG mode");
        // }

        // Optional<Field2d> field = (fieldType == FieldType.REAL_GLASS_FIELD) ? m_realGlassField : m_simulationField;
        // field.ifPresent(f -> {
        //     visionPose.ifPresentOrElse(
        //         pose -> f.getObject("VisionEstimation").setPose(pose),
        //         () -> f.getObject("VisionEstimation").setPoses()
        //     );
        // });
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
