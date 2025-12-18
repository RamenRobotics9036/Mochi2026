package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command that rotates the robot to a specific field-centric heading.
 * Uses Phoenix 6's native FieldCentricFacingAngle request.
 */
public class RotateToHeadingCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Rotation2d targetRotation;
    private final SwerveRequest.FieldCentricFacingAngle rotateRequest;

    /**
     * Creates a new RotateToHeadingCommand.
     *
     * @param drivetrain The drivetrain subsystem
     * @param targetHeadingDegrees The target field-centric heading in degrees (0 = Forward, 180 = Backward)
     */
    public RotateToHeadingCommand(CommandSwerveDrivetrain drivetrain, double targetHeadingDegrees) {
        this.drivetrain = drivetrain;
        this.targetRotation = Rotation2d.fromDegrees(targetHeadingDegrees);
        
        // Use Phoenix 6 built-in request for facing a specific angle
        this.rotateRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(targetRotation)
            // Ensure no translation while rotating
            .withVelocityX(0)
            .withVelocityY(0);
            
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setControl(rotateRequest);
    }

    @Override
    public boolean isFinished() {
        // Finish when within 2 degrees of target
        double errorDegrees = Math.abs(drivetrain.getState().Pose.getRotation().minus(targetRotation).getDegrees());
        // Normalize error to range [0, 180]? No, getDegrees() wraps -180 to 180?
        // Actually Rotation2d.minus returns a Rotation2d. getDegrees() is -180 to 180.
        // We want absolute error.
        return Math.abs(errorDegrees) < 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            // "Lock in" the target rotation to ensure gyro is perfectly synced
            // This satisfies the requirement to reset gyro after rotation
            drivetrain.resetPose(new Pose2d(
                drivetrain.getState().Pose.getTranslation(),
                targetRotation
            ));
        }
        // If interrupted, we just stop controlling. The default command will take over.
    }
}
