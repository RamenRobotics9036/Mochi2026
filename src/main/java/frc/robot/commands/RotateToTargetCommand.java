package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.visutils.TurnToAngleHelper;

import java.util.function.Supplier;

/**
 * Rotates the robot in place to face a supplied field point.
 *
 * <p>Uses CTRE's {@link SwerveRequest.FieldCentricFacingAngle} which has a
 * built-in profiled PID heading controller for silky-smooth rotation.
 * X/Y velocity is kept at zero so the robot only rotates.
 */
public class RotateToTargetCommand extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Supplier<Translation2d> m_pointSupplier;

    /** The CTRE request that drives the robot to face a target angle. */
    private final SwerveRequest.FieldCentricFacingAngle m_facingAngle =
        TurnToAngleHelper.createFacingAngleRequest();

    /** Field-coordinate X of the point to face (blue-origin, metres). */
    private double m_targetX;
    /** Field-coordinate Y of the point to face (blue-origin, metres). */
    private double m_targetY;
    /** True when the supplier returned a valid (non -1) target. */
    private boolean m_hasValidTarget;

    /**
     * Creates a new RotateToTargetCommand.
     *
     * @param drivetrain    The swerve drivetrain subsystem
     * @param pointSupplier Supplies the (x, y) field point to face.
     *                      Return x or y as -1 to indicate no target.
     */
    public RotateToTargetCommand(CommandSwerveDrivetrain drivetrain,
                                 Supplier<Translation2d> pointSupplier) {
        m_drivetrain = drivetrain;
        m_pointSupplier = pointSupplier;
        addRequirements(drivetrain);

        m_hasValidTarget = false;
    }

    @Override
    public void initialize() {
        Translation2d point = m_pointSupplier.get();
        double x = point.getX();
        double y = point.getY();

        if (x == -1 || y == -1) {
            System.out.println("no target");
            m_hasValidTarget = false;
            return;
        }

        // Copy the current values so the goal is fixed for this run
        m_targetX = x;
        m_targetY = y;
        m_hasValidTarget = true;
        System.out.println("Start rotating to target (" + m_targetX + ", " + m_targetY + ")");
    }

    @Override
    public void execute() {
        if (!m_hasValidTarget) {
            return;
        }

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
        // End immediately if there was no valid target
        if (!m_hasValidTarget) {
            return true;
        }
        // Otherwise, run as long as the button is held (whileTrue binding)
        return false;
    }
}
