package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Optional;

/**
 * Rotates the robot in place to face a specific AprilTag.
 *
 * <p>Uses CTRE's {@link SwerveRequest.FieldCentricFacingAngle} which has a
 * built-in profiled PID heading controller for silky-smooth rotation.
 * X/Y velocity is kept at zero so the robot only rotates.
 */
public class TurnToAprilTagCommand extends Command {

    private static final AprilTagFieldLayout kFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final int m_tagId;

    /** The CTRE request that drives the robot to face a target angle. */
    private final SwerveRequest.FieldCentricFacingAngle m_facingAngle =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);

    /** Pre-computed target angle (robot-to-tag bearing). */
    private Rotation2d m_targetAngle = Rotation2d.kZero;

    /**
     * Creates a new TurnToAprilTagCommand.
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param tagId      The AprilTag ID to face
     */
    public TurnToAprilTagCommand(CommandSwerveDrivetrain drivetrain, int tagId) {
        m_drivetrain = drivetrain;
        m_tagId = tagId;
        addRequirements(drivetrain);

        // Tune the heading PID — these give smooth, non-oscillatory convergence.
        // The controller output is in rad/s; P of 8 means 8 rad/s per radian of error.
        m_facingAngle.HeadingController.setPID(8, 0, 0.5);
        // Enable continuous input so it takes the shortest path across the ±180° wrap.
        m_facingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Look up the tag pose on the field
        Optional<Pose3d> tagPose = kFieldLayout.getTagPose(m_tagId);
        if (tagPose.isEmpty()) {
            System.err.println("TurnToAprilTag: Tag ID " + m_tagId + " not found in field layout!");
            m_targetAngle = Rotation2d.kZero;
            return;
        }

        // Print x,y of tagPose
        System.out.println("TurnToAprilTag: Tag " + m_tagId + " pose: x=" +
            tagPose.get().getX() + " y=" + tagPose.get().getY());

        // Compute bearing from the robot to the tag
        Pose2d robotPose = m_drivetrain.getState().Pose;
        double dx = tagPose.get().getX() - robotPose.getX();
        double dy = tagPose.get().getY() - robotPose.getY();
        m_targetAngle = new Rotation2d(dx, dy);
    }

    @Override
    public void execute() {
        // Re-compute the bearing each cycle so the heading tracks if the odometry
        // pose drifts slightly while rotating.
        Optional<Pose3d> tagPose = kFieldLayout.getTagPose(m_tagId);
        if (tagPose.isPresent()) {
            Pose2d robotPose = m_drivetrain.getState().Pose;
            double dx = tagPose.get().getX() - robotPose.getX();
            double dy = tagPose.get().getY() - robotPose.getY();
            m_targetAngle = new Rotation2d(dx, dy);
        }

        // Convert from field (blue-origin) frame to operator-perspective frame
        Rotation2d operatorAngle = toOperatorFrame(m_targetAngle);

        // Print target angle
        System.out.println("TurnToAprilTag: Target angle to tag " + m_tagId + " is " +
            m_targetAngle.getDegrees() + " deg (field), " +
            operatorAngle.getDegrees() + " deg (operator)");

        // Drive with zero translation, facing the computed angle
        m_drivetrain.setControl(
            m_facingAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(operatorAngle)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion when the command ends
        m_drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        // Run as long as the button is held (whileTrue binding)
        return false;
    }

    /**
     * Converts an absolute field-frame angle (0° = toward red wall) into the
     * operator-perspective frame used by {@link SwerveRequest.FieldCentricFacingAngle}.
     *
     * <p>On blue alliance the operator forward is 0° so this is a no-op.
     * On red alliance the operator forward is 180°, so this subtracts 180°.
     * The result is automatically normalized to (−180°, 180°] by {@link Rotation2d},
     * and the heading controller's continuous-input wrapping handles the rest.
     */
    private Rotation2d toOperatorFrame(Rotation2d fieldAngle) {
        return fieldAngle.minus(m_drivetrain.getOperatorForwardDirection());
    }
}
