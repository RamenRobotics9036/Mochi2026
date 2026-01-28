package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.auto.DriveForwardNow;

import java.util.ArrayList;
import java.util.List;

/**
 * Static utility class that manages autonomous selection and execution logic.
 * 
 * <p>This class handles the interface between PathPlanner, the Shuffleboard dashboard,
 * and the physical drivetrain to provide a centralized way to choose and run 2026 auto routines.
 */
@SuppressWarnings("all")
public final class  AutoLogic {

    /** The UI element for selecting autonomous routines on the dashboard. */
    public static final SendableChooser<String> autoPicker = new SendableChooser<>();
    
    /** The Shuffleboard tab dedicated to autonomous settings and feedback. */
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Autos");

    /** Static reference to the drivetrain to allow manual command generation. */
    private static CommandSwerveDrivetrain m_drivetrain;

    /** Constant name for the manual backup autonomous routine. */
    private static final String K_MANUAL_DRIVE_NAME = "MANUAL: Drive 2m Forward";

    private AutoLogic() {
        throw new UnsupportedOperationException("Static utility class!");
    }

    /** 
     * Registers PathPlanner configurations and warms up commands to prevent lag during the match.
     */
    public static void registerCommands() {
        try {
            FollowPathCommand.warmupCommand().schedule();
        } catch (Exception e) {
            DriverStation.reportWarning("Autonomous warmup failed: " + e.getMessage(), false);
        }
    }

    /** 
     * Initializes the Shuffleboard tab with the auto chooser and active status.
     * 
     * @param drivetrain The drivetrain instance required for manual auto routines.
     */
    public static void initShuffleboard(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addAutoOptions();

        tab.add("Auto Selector", autoPicker)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);

        tab.addString("Active Auto", AutoLogic::getSelectedName)
            .withPosition(0, 1)
            .withSize(2, 1);
    }

    /** Adds both PathPlanner routines and hard-coded manual routines to the chooser. */
    private static void addAutoOptions() {
        // Manual Autos
        autoPicker.addOption("Manual: Drive 2m Forward", K_MANUAL_DRIVE_NAME);
        
        // Pathplanner Autos
        autoPicker.setDefaultOption("Center Auto", "Center Auto");
        autoPicker.addOption("Scale test", "Scale test");
        autoPicker.addOption("Diagonal path", "Diagonal path");   
    }

    /**
     * Factory method that returns the Command corresponding to the provided string name.
     * 
     * @param autoName The name of the routine as defined in PathPlanner or local constants.
     * @return A command ready to be scheduled; {@link Commands#none()} if name is null or invalid.
     */
    public static Command getAutoCommand(String autoName) {
        if (autoName == null) return Commands.none();

        // Check for manual code-based routines first
        if (autoName.equals(K_MANUAL_DRIVE_NAME)) {
            return new DriveForwardNow(m_drivetrain, 2.0, true).withName("ManualDriveForward");
        }

        // Build PathPlanner GUI routines
        try {
            return AutoBuilder.buildAuto(autoName).withName(autoName);
        } catch (Exception e) {
            DriverStation.reportError("Failed to build autonomous: " + autoName, e.getStackTrace());
            return Commands.none();
        }
    }

    /**
     * Wraps the selected auto command with haptic feedback upon completion.
     * 
     * @param controller The Xbox controller to vibrate when auto finishes.
     * @return The sequence including the auto routine and a 0.5s rumble.
     */
    public static Command getSelectedAutoCommandWithFeedback(CommandXboxController controller) {
        String autoName = getSelectedName();
        return getAutoCommand(autoName).andThen(
            Commands.startEnd(
                () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.6),
                () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0)
            ).withTimeout(0.5)
        ).withName(autoName + "_WithRumble");
    }

    /** Returns the string currently selected in the dashboard chooser. */
    public static String getSelectedName() {
        return autoPicker.getSelected();
    }

    /** Retrieves the command for the currently selected dashboard option. */
    public static Command getSelectedAutoCommand() {
        return getAutoCommand(getSelectedName());
    }

    /** 
     * Generates a trajectory profile for the selected auto for use in field visualization.
     */
    public static AutoTrajectoryProfile getSelectedAutoProfile() {
        String autoName = getSelectedName();
        if (autoName == null || autoName.equals(K_MANUAL_DRIVE_NAME)) return null;

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(autoName);
            return new AutoTrajectoryProfile(List.of(path));
        } catch (Exception e) {
            return null;
        }
    }
}

/** 
 * Helper class to calculate trajectory timing and starting positions.
 * Used primarily for dashboard visualization and pre-match planning.
 */
class AutoTrajectoryProfile {
    /** The coordinate where the robot expects to be placed on the field. */
    private final Pose2d startingPose;
    
    /** List of individual path segments that make up the routine. */
    private final List<PathPlannerTrajectory> trajectories;
    
    /** Total predicted time in seconds to complete the autonomous routine. */
    private final double autoDuration;

    // Constructs the trajectory profile from a list of PathPlanner paths.
    public AutoTrajectoryProfile(List<PathPlannerPath> paths) {
        Pose2d initialPose = new Pose2d();
        List<PathPlannerTrajectory> trajs = new ArrayList<>();
        double totalTime = 0.0;

        if (paths != null && !paths.isEmpty()) {
            initialPose = paths.get(0).getStartingHolonomicPose().orElse(new Pose2d());

            try {
                // Load robot physics configuration from GUI settings
                RobotConfig config = RobotConfig.fromGUISettings();
                ChassisSpeeds speeds = new ChassisSpeeds();
                Rotation2d rotation = initialPose.getRotation();
                // Build each trajectory segment sequentially   
                for (PathPlannerPath path : paths) {
                    if (path != null) {
                        PathPlannerTrajectory traj = new PathPlannerTrajectory(path, speeds, rotation, config);
                        trajs.add(traj);
                        speeds = traj.getEndState().fieldSpeeds;
                        rotation = traj.getEndState().pose.getRotation();
                    }
                }
                totalTime = trajs.stream().mapToDouble(t -> t.getTotalTimeSeconds()).sum();
            } catch (Exception e) {
                DriverStation.reportError("RobotConfig Error during profile generation: " + e.getMessage(), e.getStackTrace());
            }
        }

        this.startingPose = initialPose;
        this.trajectories = List.copyOf(trajs);
        this.autoDuration = totalTime;
    }

    /** @return The initial pose required for this auto routine. */
    public Pose2d getStartingPose() { return startingPose; }
    
    /** @return The list of generated trajectories for simulation. */
    public List<PathPlannerTrajectory> getTrajectories() { return trajectories; }
    
    /** @return Total time in seconds the routine will take to execute. */
    public double getRunTimeSeconds() { return autoDuration; }
}