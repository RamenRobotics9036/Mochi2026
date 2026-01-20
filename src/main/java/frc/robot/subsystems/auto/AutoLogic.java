package frc.robot.subsystems.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("all")
public final class AutoLogic {

    public static final SendableChooser<String> autoPicker = new SendableChooser<>();
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Autos");

    private AutoLogic() {
        throw new UnsupportedOperationException("Static utility class!");
    }

    /** 
     * Registers PathPlanner configurations and warms up commands.
     */
    public static void registerCommands() {

        try {
            CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        } catch (Exception e) {
            DriverStation.reportWarning("Autonomous warmup failed: " + e.getMessage(), false);
        }
    }

    /** Setup the Shuffleboard dashboard */
    public static void initShuffleboard() {
        addAutoOptions();

        tab.add("Auto Selector", autoPicker)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);

        tab.addString("Active Auto", AutoLogic::getSelectedName)
            .withPosition(0, 1)
            .withSize(2, 1);
    }

    private static void addAutoOptions() {
        autoPicker.setDefaultOption("Center Auto", "Birdi Auto Center");
        autoPicker.addOption("LEFT 1 Coral", "auto 1 coral left");
        autoPicker.addOption("RIGHT 1 Coral", "auto 1 coral right");
    }

    public static Command getAutoCommand(String autoName) {
        if (autoName == null) return Commands.none();
        try {
            return AutoBuilder.buildAuto(autoName).withName(autoName);
        } catch (FileVersionException e) {
            DriverStation.reportError("Failed to build autonomous: " + autoName, e.getStackTrace());
            return Commands.none();
        }
    }

    /**
     * Vibrates controller for 0.5s after the auto routine finishes
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

    public static String getSelectedName() {
        return autoPicker.getSelected();
    }

    public static Command getSelectedAutoCommand() {
        return getAutoCommand(getSelectedName());
    }

    public static AutoTrajectoryProfile getSelectedAutoProfile() {
        String autoName = getSelectedName();
        if (autoName == null) return null;

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(autoName);
            return new AutoTrajectoryProfile(List.of(path));
        } catch (Exception e) {
            return null;
        }
    }
}

/** 
 * Trajectory Profile
 * Calculates exactly how long the auto will take and where it starts.
 */
class AutoTrajectoryProfile {
    private final Pose2d startingPose;
    private final List<PathPlannerTrajectory> trajectories;
    private final double autoDuration;

    public AutoTrajectoryProfile(List<PathPlannerPath> paths) {
        Pose2d initialPose = new Pose2d();
        List<PathPlannerTrajectory> trajs = new ArrayList<>();
        double totalTime = 0.0;

        if (paths != null && !paths.isEmpty()) {
            initialPose = paths.get(0).getStartingHolonomicPose().orElse(new Pose2d());

            try {
                RobotConfig config = RobotConfig.fromGUISettings();
                ChassisSpeeds speeds = new ChassisSpeeds();
                Rotation2d rotation = initialPose.getRotation();

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
                DriverStation.reportError("RobotConfig Error: " + e.getMessage(), e.getStackTrace());
            }
        }

        this.startingPose = initialPose;
        this.trajectories = List.copyOf(trajs);
        this.autoDuration = totalTime;
    }

    public Pose2d getStartingPose() { return startingPose; }
    public List<PathPlannerTrajectory> getTrajectories() { return trajectories; }
    public double getRunTimeSeconds() { return autoDuration; }
}