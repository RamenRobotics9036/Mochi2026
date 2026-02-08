package frc.robot.visutils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.auto.AutoLogic;
import java.util.Optional;


/**
 * Manages the tape-drop accuracy test workflow.
 *
 * <p>Drops blue tape at the Kalman-estimated pose before an auto runs,
 * then drops red tape at the Kalman-estimated pose after, so you can
 * compare where the robot thought it was vs. where it ended up.
 */
public class DriveAccuracyTester {

    /** Pose of the blue tape on the field, or empty to hide. */
    private Optional<Pose2d> m_blueTapePose = Optional.of(new Pose2d(1.0, 1.0, new Rotation2d()));

    /** Pose of the red tape on the field, or empty to hide. */
    private Optional<Pose2d> m_redTapePose = Optional.of(new Pose2d(2.0, 1.0, new Rotation2d()));

    /** The drive subsystem. */
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_driveSubsystem;

    /** Vision-only Kalman filter for precise stationary position estimation. */
    private final VisionKalmanFilter m_visionKalmanFilter;

    /**
     * @param driveSubsystem The drive subsystem
     * @param visionKalmanFilter The Kalman filter used to get converged pose estimates
     */
    public DriveAccuracyTester(
        SwerveDrivetrain<TalonFX, TalonFX, CANcoder> driveSubsystem,
        VisionKalmanFilter visionKalmanFilter) {

        if (driveSubsystem == null) {
            throw new IllegalArgumentException("driveSubsystem cannot be null");
        }

        m_driveSubsystem = driveSubsystem;
        m_visionKalmanFilter = visionKalmanFilter;
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
    private Pose2d computeFrontTapePose(Pose2d robotPose) {
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
    }

    private void cleanupAfterTest() {
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
        return Commands.either(
            // Converged path: check that the auto is relative (no fixed starting pose)
            Commands.either(
                // Relative auto: clear tape, drop blue tape, pause, run auto, drop red tape
                Commands.sequence(
                    Commands.runOnce(this::clearTape),
                    Commands.runOnce(() -> {
                        Pose2d kalmanPose = m_visionKalmanFilter.getEstimate();
                        m_blueTapePose = Optional.of(computeFrontTapePose(kalmanPose));
                        System.out.println("Blue tape dropped at Kalman estimate: " + m_blueTapePose.get());
                    }),
                    Commands.waitSeconds(1.0),
                    AutoLogic.getSelectedAutoCommand(),

                    Commands.print("Waiting for Camera to lock on again..."),
                    Commands.waitUntil(m_visionKalmanFilter::hasConverged).withTimeout(10),
                    Commands.either(
                        Commands.runOnce(() -> {
                            Pose2d kalmanPose = m_visionKalmanFilter.getEstimate();
                            m_redTapePose = Optional.of(computeFrontTapePose(kalmanPose));
                            System.out.println("Red tape dropped at Kalman estimate: " + m_redTapePose.get());
                        }),
                        Commands.print("Failed to lock onto camera at end of cycle."),
                        m_visionKalmanFilter::hasConverged
                    )
                ),
                // Auto has a fixed starting pose — not supported
                Commands.print("Only relative auto paths are supported."),
                // Condition: auto has no starting pose (is relative)
                () -> AutoLogic.getSelectedAutoStartingPose().equals(Pose2d.kZero)
            ),
            // Not converged path: print warning and do nothing
            Commands.runOnce(() ->
                System.out.println("Kalman filter not converged — tape drop auto aborted")),
            // Condition: is the Kalman filter converged?
            m_visionKalmanFilter::hasConverged
        ).beforeStarting(this::initializeTest)
         .finallyDo(this::cleanupAfterTest);
    }
}
