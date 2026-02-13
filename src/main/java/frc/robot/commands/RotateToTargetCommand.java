package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.visutils.TurnToAngleHelper;

import java.util.Optional;

/**
 * Rotates the robot in place to face a specific AprilTag.
 *
 * <p>Uses CTRE's {@link SwerveRequest.FieldCentricFacingAngle} which has a
 * built-in profiled PID heading controller for silky-smooth rotation.
 * X/Y velocity is kept at zero so the robot only rotates.
 */
public class RotateToTargetCommand extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final int m_tagId;

    /** The CTRE request that drives the robot to face a target angle. */
    private final SwerveRequest.FieldCentricFacingAngle m_facingAngle =
        TurnToAngleHelper.createFacingAngleRequest();

    /** Field-coordinate X of the point to face (blue-origin, metres). */
    private double m_targetX;
    /** Field-coordinate Y of the point to face (blue-origin, metres). */
    private double m_targetY;

    /**
     * Creates a new RotateToTargetCommand.
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param tagId      The AprilTag ID to face
     */
    public RotateToTargetCommand(CommandSwerveDrivetrain drivetrain, int tagId) {
        m_drivetrain = drivetrain;
        m_tagId = tagId;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("Start rotating to target tag ID " + m_tagId);

        // Resolve the tag ID to a field coordinate once at start
        Optional<Pose3d> tagPose = TurnToAngleHelper.getTagPose(m_tagId);
        if (tagPose.isEmpty()) {
            System.err.println("RotateToTarget: Tag ID " + m_tagId
                + " not found in field layout!");
            m_targetX = 0;
            m_targetY = 0;
            return;
        }

        m_targetX = tagPose.get().getX();
        m_targetY = tagPose.get().getY();
    }

    @Override
    public void execute() {
        // Re-compute the bearing each cycle so the heading tracks as the
        // robot moves (the target point stays fixed).
        Rotation2d operatorAngle =
            TurnToAngleHelper.bearingToPointInOperatorFrame(
                m_targetX, m_targetY,
                m_drivetrain.getState().Pose,
                m_drivetrain.getOperatorForwardDirection());

        // Print target angle
        // System.out.println("RotateToTarget: Target angle to ("
        //     + m_targetX + ", " + m_targetY + ") is "
        //     + operatorAngle.getDegrees() + " deg (operator)");

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

        System.out.println("Stopped rotating.");
    }

    @Override
    public boolean isFinished() {
        // Run as long as the button is held (whileTrue binding)
        return false;
    }
}
