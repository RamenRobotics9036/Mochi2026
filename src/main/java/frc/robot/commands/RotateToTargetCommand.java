package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.visutils.TurnToAngleHelper;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Rotates the robot in place to face a supplied field point.
 *
 * <p>Uses CTRE's {@link SwerveRequest.FieldCentricFacingAngle} which has a
 * built-in profiled PID heading controller for silky-smooth rotation.
 * X/Y velocity is kept at zero so the robot only rotates.
 */
public class RotateToTargetCommand extends Command {
    /** The subsystem for the swerve drivetrain. */
    private final CommandSwerveDrivetrain m_drivetrain;
    /** The position on the field to rotate towards. */
    private final Supplier<Optional<Translation2d>> m_pointSupplier;

    /** The CTRE request that drives the robot to face a target angle. */
    private final SwerveRequest.FieldCentricFacingAngle m_facingAngle =
        TurnToAngleHelper.createFacingAngleRequest();

    /** The field point to face (blue-origin, metres), or empty if no target. */
    private Optional<Translation2d> m_target = Optional.empty();

    /**
     * Creates a new RotateToTargetCommand.
     *
     * @param drivetrain    The swerve drivetrain subsystem
     * @param pointSupplier Supplies the (x, y) field point to face,
     *                      or empty if no target is available.
     */
    public RotateToTargetCommand(CommandSwerveDrivetrain drivetrain,
                                 Supplier<Optional<Translation2d>> pointSupplier) {
        m_drivetrain = drivetrain;
        m_pointSupplier = pointSupplier;
        addRequirements(drivetrain);
    }

    /** Gets the rotation target, ensures it's valid, and then logs its result. */
    @Override
    public void initialize() {
        m_target = m_pointSupplier.get();

        if (m_target.isEmpty()) {
            System.out.println("no target");
        } else {
            System.out.println("Start rotating to target (" + m_target.get().getX() + ", " + m_target.get().getY() + ")");
        }
    }

    /** Checks the relative rotation of the target and handles the rotation of the robot accordingly. */
    @Override
    public void execute() {
        if (m_target.isEmpty()) {
            return;
        }

        // Re-compute the bearing each cycle so the heading tracks as the
        // robot moves (the target point stays fixed).
        Translation2d target = m_target.get();
        Rotation2d operatorAngle =
            TurnToAngleHelper.bearingToPointInOperatorFrame(
                target.getX(), target.getY(),
                m_drivetrain.getState().Pose,
                m_drivetrain.getOperatorForwardDirection());

        // Drive with zero translation, facing the computed angle
        m_drivetrain.setControl(
            m_facingAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(operatorAngle)
        );
    }

    /** Stop all rotation when the command ends. */
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.Idle());

        System.out.println("Stopped rotating.");
    }

    /**
     *  End immediately if there was no valid target;
     *  otherwise run as long as the button is held (whileTrue binding)
     */
    @Override
    public boolean isFinished() {
        return m_target.isEmpty();
    }
}
