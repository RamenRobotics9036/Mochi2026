package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.visutils.LimelightOdometry;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import java.util.function.DoubleSupplier;

/**
 * Command for vision testing in sim
 * It is designed to be set as the default command for the {@link VisionSubsystem}.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class TestingCommand extends Command {
    private CommandSwerveDrivetrain m_drivetrain;
    private FieldCentricFacingAngle m_rotationRequest = new FieldCentricFacingAngle();

    /**
     * Creates a new VisonDefaultCommand.
     *
     * @param vision The vision subsystem to be supplied
     */
    public TestingCommand(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        m_drivetrain.setControl(m_rotationRequest.withTargetDirection(Rotation2d.fromDegrees(53)));
    }

    /**
     * This command currently isn't complete so it doesn't
     * have a finished state.
     * It stops if another request is applied.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
