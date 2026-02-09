package frc.robot.visutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.auto.AutoLogic;
import java.util.Optional;
import java.util.function.Consumer;


/**
 * Manages the tape-drop accuracy test workflow.
 *
 * <p>Drops blue tape at the Kalman-estimated pose before an auto runs,
 * then drops red tape at the Kalman-estimated pose after, so you can
 * compare where the robot thought it was vs. where it ended up.
 */
public class DriveAccuracyTester {

    /** Pose of the blue tape on the field, or empty to hide. */
    private Optional<Pose2d> m_blueTapePose = Optional.empty();

    /** Pose of the red tape on the field, or empty to hide. */
    private Optional<Pose2d> m_redTapePose = Optional.empty();

    /** The drive subsystem. */
    private final CommandSwerveDrivetrain m_driveSubsystem;

    /** Vision-only Kalman filter for precise stationary position estimation. */
    private final VisionKalmanFilter m_visionKalmanFilter;

    /** Callback to force-disable or re-enable vision injection into odometry. */
    private final Consumer<Boolean> m_forceDisableVision;

    /**
     * @param driveSubsystem The drive subsystem
     * @param visionKalmanFilter The Kalman filter used to get converged pose estimates
     * @param forceDisableVision Callback to force-disable (true) or re-enable (false) vision
     */
    public DriveAccuracyTester(
        CommandSwerveDrivetrain driveSubsystem,
        VisionKalmanFilter visionKalmanFilter,
        Consumer<Boolean> forceDisableVision) {

        if (driveSubsystem == null) {
            throw new IllegalArgumentException("driveSubsystem cannot be null");
        }

        m_driveSubsystem = driveSubsystem;
        m_visionKalmanFilter = visionKalmanFilter;
        m_forceDisableVision = forceDisableVision;
    }

    /** @return Current blue tape pose, or empty if hidden */
    public Optional<Pose2d> getBlueTapePose() {
        return m_blueTapePose;
    }

    /** @return Current red tape pose, or empty if hidden */
    public Optional<Pose2d> getRedTapePose() {
        return m_redTapePose;
    }

    /** Clears both tape poses so they are no longer displayed on the field. */
    public void clearTape() {
        m_blueTapePose = Optional.empty();
        m_redTapePose = Optional.empty();
    }

    /**
     * Computes a tape pose at the front of the robot, parallel to the front face.
     * The tape is placed just beyond the front edge so it doesn't overlap the robot.
     *
     * @param robotPose The robot's current pose
     * @return The tape Pose2d positioned in front of the robot
     */
    public Pose2d computeFrontTapePose(Pose2d robotPose) {
        // Half the bumper length (center to front edge): 0.864m / 2 = 0.432m
        // Then push out by the tape width so it doesn't overlap the bumpers
        double halfBumperLength = 0.864 / 2.0;
        double tapeWidth = 0.137;
        double frontOffset = halfBumperLength + (tapeWidth / 2);

        // Transform to place tape in front of the robot, rotated 90° so the
        // tape's long axis is parallel to the robot's front face
        return robotPose.transformBy(
            new Transform2d(frontOffset, 0, Rotation2d.fromDegrees(90)));
    }

    private void initializeTest() {
        // Disable vision injection into driveTrain.  We want the path to drive PURELY based on
        // odometry, so we can see how far off it is at the end of the path.
        m_forceDisableVision.accept(true);
    }

    private void cleanupAfterTest() {
        // Always re-enable vision (which we temporarily disabled)
        m_forceDisableVision.accept(false);
    }

    /**
     * Creates a command sequence that:
     * 1) Checks Kalman filter convergence (aborts if not converged)
     * 2) Checks the auto is relative (no fixed starting pose)
     * 3) Drops blue tape at the Kalman-estimated front pose
     * 4) Runs the currently selected auto
     * 5) Waits for Kalman re-convergence, then drops red tape
     *
     * @return The composed command
     */
    public Command createTapeDropAutoCommand() {
        Command cmd = runTestSequence()
            .onlyIf(this::checkPreconditions)
            .beforeStarting(this::initializeTest)
            .finallyDo(this::cleanupAfterTest);

        cmd.addRequirements(m_driveSubsystem);
        return cmd;
    }

    /**
     * Checks all preconditions before running the test. Prints a message and
     * returns false to abort if any check fails.
     */
    private boolean checkPreconditions() {
        if (!m_visionKalmanFilter.hasConverged()) {
            System.out.println("Kalman filter not converged, tape drop auto aborted.");
            return false;
        }
        if (!AutoLogic.getSelectedAutoStartingPose().equals(Pose2d.kZero)) {
            System.out.println("Only relative auto paths are supported.");
            return false;
        }
        return true;
    }

    /** Clear tape → blue tape → pause → auto → wait for convergence → red tape. */
    private Command runTestSequence() {
        return Commands.sequence(
            Commands.runOnce(this::clearTape),
            dropBlueTape(),
            Commands.waitSeconds(1.0),
            AutoLogic.getSelectedAutoCommand(),
            waitForConvergenceAndDropRedTape()
        );
    }

    /** Drops blue tape at the Kalman-estimated front pose. */
    private Command dropBlueTape() {
        return Commands.runOnce(() -> {
            Pose2d kalmanPose = m_visionKalmanFilter.getEstimate();
            m_blueTapePose = Optional.of(computeFrontTapePose(kalmanPose));
            System.out.println("Blue tape dropped at Kalman estimate: " + m_blueTapePose.get());
        });
    }

    /** Drops red tape at the Kalman-estimated front pose. */
    private Command dropRedTape() {
        return Commands.runOnce(() -> {
            Pose2d kalmanPose = m_visionKalmanFilter.getEstimate();
            m_redTapePose = Optional.of(computeFrontTapePose(kalmanPose));
            System.out.println("Red tape dropped at Kalman estimate: " + m_redTapePose.get());
        });
    }

    /** Waits up to 10 s for the Kalman filter to re-converge, then drops red tape or prints failure. */
    private Command waitForConvergenceAndDropRedTape() {
        return Commands.sequence(
            Commands.print("Waiting for Camera to lock on again..."),
            Commands.waitUntil(m_visionKalmanFilter::hasConverged).withTimeout(10),
            Commands.either(
                dropRedTape(),
                Commands.print("Failed to lock onto camera at end of cycle."),
                m_visionKalmanFilter::hasConverged
            )
        );
    }
}
