package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.visutils.AllianceCalc;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.IntSupplier;

/**
 * Fixed-field-pose autonomous drive routine for climb preparation.
 *
 * <p>The robot drives to the fixed field pose at x=15.27, y=5.22,
 * heading=179.8° and then stops.
 * The target pose will be flipped for the opposite alliance.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class AutonomousClimbVisionAlign extends Command {
    private static final Pose2d kTargetPoseBlueAlliance = new Pose2d(
        15.27,
        5.22,
        Rotation2d.fromDegrees(179.8)
    );
    private Pose2d m_targetPose = kTargetPoseBlueAlliance;
    private static final double kPositionToleranceMeters = 0.05;
    private static final double kHeadingToleranceRadians = Math.toRadians(1.5);
    private static final double kMaxTranslationSpeedMetersPerSecond = 1.0;
    private static final double kMaxRotationSpeedRadiansPerSecond = Math.PI; // ~180 deg/s
    private static final double kTranslationPGain = 1.0;
    private static final double kRotationPGain = 1.0;

    private final BotConfigInterface m_configInterface;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final IntSupplier m_climbTagIdSupplier;

    private final RobotCentric m_robotCentricRequest = new RobotCentric();
    private final Timer m_timer = new Timer();
    // unused fields from the previous vision-alignment implementation
    // private double m_previousHeading = 0.0;
    // private double m_accumulatedRotation = 0.0;

    private boolean m_alignmentFailed = false;
    private enum AutoState {
        DRIVE_TO_TARGET,
        DONE
    }

    private AutoState m_state = AutoState.DRIVE_TO_TARGET;

    public AutonomousClimbVisionAlign(
            BotConfigInterface configInterface,
            CommandSwerveDrivetrain drivetrain,
            IntSupplier climbTagIdSupplier
    ) {
        m_configInterface = configInterface;
        m_drivetrain = drivetrain;
        m_climbTagIdSupplier = climbTagIdSupplier;
        // this.m_limelightName = m_configInterface.getCameraName(0);
        addRequirements(drivetrain);
    }

    /** Convenience ctor for fixed tag ID. */
    public AutonomousClimbVisionAlign(
            BotConfigInterface configInterface,
            CommandSwerveDrivetrain drivetrain,
            int climbTagId
    ) {
        this(configInterface, drivetrain, () -> climbTagId);
    }

    @Override
    public void initialize() {
        m_alignmentFailed = false;
        m_timer.reset();
        m_timer.start();
        m_state = AutoState.DRIVE_TO_TARGET;
        m_targetPose = AllianceCalc.isRedAlliance()
            ? kTargetPoseBlueAlliance
            : AllianceCalc.flipFieldPose(kTargetPoseBlueAlliance);
        System.out.println(String.format(
            "[AutonomousClimbVisionAlign] Target pose: x=%.2f, y=%.2f, heading=%.1f°",
            m_targetPose.getX(),
            m_targetPose.getY(),
            m_targetPose.getRotation().getDegrees()));
        System.out.println("[AutonomousClimbVisionAlign] Starting fixed-pose drive routine");
    }

    @Override
    public void execute() {
        if (m_alignmentFailed) {
            return;
        }

        Pose2d currentPose = m_drivetrain.getState().Pose;
        System.out.println(String.format(
            "[AutonomousClimbVisionAlign] Current pose: x=%.2f, y=%.2f, heading=%.1f°",
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().getDegrees()));
        switch (m_state) {
            case DRIVE_TO_TARGET -> {
                Pose2d robotRelativeTarget = m_targetPose.relativeTo(currentPose);
                double errorX = robotRelativeTarget.getX();
                double errorY = robotRelativeTarget.getY();
                double distanceToTarget = Math.hypot(errorX, errorY);
                double headingError = MathUtil.inputModulus(
                    m_targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians(),
                    -Math.PI,
                    Math.PI
                );

                System.out.println(String.format(
                    "[AutonomousClimbVisionAlign] target=(%.2f, %.2f, %.1f°) error=(%.2f, %.2f, %.1f°)",
                    m_targetPose.getX(),
                    m_targetPose.getY(),
                    m_targetPose.getRotation().getDegrees(),
                    errorX,
                    errorY,
                    Math.toDegrees(headingError)));

                if (distanceToTarget <= kPositionToleranceMeters
                    && Math.abs(headingError) <= kHeadingToleranceRadians) {
                    m_state = AutoState.DONE;
                    System.out.println("[AutonomousClimbVisionAlign] Reached target pose, stopping robot");
                } else {
                    double desiredVX = MathUtil.clamp(
                        errorX * kTranslationPGain,
                        -kMaxTranslationSpeedMetersPerSecond,
                        kMaxTranslationSpeedMetersPerSecond);
                    double desiredVY = MathUtil.clamp(
                        errorY * kTranslationPGain,
                        -kMaxTranslationSpeedMetersPerSecond,
                        kMaxTranslationSpeedMetersPerSecond);
                    double desiredOmega = MathUtil.clamp(
                        headingError * kRotationPGain,
                        -kMaxRotationSpeedRadiansPerSecond,
                        kMaxRotationSpeedRadiansPerSecond);
                    m_drivetrain.setControl(m_robotCentricRequest
                        .withVelocityX(desiredVX)
                        .withVelocityY(desiredVY)
                        .withRotationalRate(desiredOmega));
                }
            }
            case DONE -> {
                m_drivetrain.setControl(new SwerveRequest.Idle());
            }
        }
    }

    // No vision logging or pose calculation required for this simple fixed-pose drive command.

    @Override
    public boolean isFinished() {
        if (m_alignmentFailed) {
            return true;
        }

        if (m_timer.hasElapsed(15.0)) {
            System.out.println("[AutonomousClimbVisionAlign] Timed out after 15 seconds");
            return true;
        }
        return m_state == AutoState.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.Idle());
        if (interrupted) {
            System.out.println("[AutonomousClimbVisionAlign] Interrupted");
        }
    }

    // No unused target pose or vision search methods required in this simplified routine.
}