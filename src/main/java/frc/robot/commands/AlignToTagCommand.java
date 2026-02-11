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
public class AlignToTagCommand extends Command {
    private CommandSwerveDrivetrain m_drivetrain;
    private String m_limelightName;
    private CommandXboxController m_joystick;
    private Rotation2d m_targetRotation;
    private final SwerveRequest.RobotCentric m_driveRequest = new SwerveRequest.RobotCentric();
    private boolean isComplete = false;
    private double m_targetRotationDegrees;
    private double m_rotationDirection;

    /**
     * Creates a new VisionDefaultCommand.
     *
     * @param vision The vision subsystem to be supplied
     */
    public AlignToTagCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
        m_drivetrain = drivetrain;
        m_joystick = joystick;

        if (Robot.isSimulation()) {
            m_limelightName = VisionConstants.kLimelightNameSim;
        }
        else {
            m_limelightName = VisionConstants.kLimelightNameReal;
        }

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        /** Ensures the limelight is even locked on to any AprilTags to align to */
        if (LimelightHelpers.getFiducialID(m_limelightName) == -1.0) {
            isComplete = true;
            return;
        }

        /** If so, calculate the desired rotation */
        double targetRotationDegrees = LimelightHelpers.getTX(m_limelightName);
        SwerveDriveState driveState = m_drivetrain.getState();
        Pose2d driveStatePose = driveState.Pose;
        
        /** Gets the desired rotation using the relative
         * rotation needed and the current Rotation2D */
        m_targetRotation = Rotation2d.fromDegrees (
            driveStatePose.getRotation().getDegrees()
            + targetRotationDegrees
        );

        m_rotationDirection = Math.signum(targetRotationDegrees);
        m_targetRotationDegrees = m_targetRotation.getDegrees();
    }

    @Override
    public void execute() {
        SwerveDriveState driveState = m_drivetrain.getState();
        Pose2d driveStatePose = driveState.Pose;
        double rotationOffset = driveStatePose.getRotation().getDegrees() - m_targetRotationDegrees;

        if (
            Math.abs(rotationOffset) < TunerConstants.kAlignmentErrorMargin
        ) {
            System.out.println(rotationOffset);
            isComplete = true;
        }
        else {
            m_drivetrain.setControl(m_driveRequest.withRotationalRate(3.0));
        }
    }

    /**
     * This command currently isn't complete so it doesn't
     * have a finished state.
     * It stops if another request is applied.
     */
    @Override
    public boolean isFinished() {
        /** Checks to see if the command is done */
        if (isComplete) {
            System.out.println("Alignment to AprilTag complete!");
            return true;
        }

        /** Gets the absolute value of the user's left joystick input */
        double absoluteMoveInput = Math.abs(
            /** Gets the total value of the user's left joystick input
             * using the Pythagorean Theorem */
            Math.sqrt(
                /** x^2 + y^2 */
                Math.pow(m_joystick.getLeftX(), 2)
                + Math.pow(m_joystick.getLeftY(), 2)
            )
        );

        /** Gets the absolute value of the user's right joystick input */
        double absoluteTurnInput = Math.abs(
            /** Gets the total value of the user's right joystick input
             * using the Pythagorean Theorem */
            Math.sqrt(
                /** x^2 + y^2 */
                Math.pow(m_joystick.getRightX(), 2)
                + Math.pow(m_joystick.getRightY(), 2)
            )
        );

        /** Stops if the user tries to manually move the robot */
        if (absoluteMoveInput > TunerConstants.kSwerveMoveInterruptionSensitivity) {
            return true;
        }
        /** Stops if the user tries to manually turn the robot */
        else if (absoluteTurnInput > TunerConstants.kSwerveTurnInterruptionSensitivity) {
            return true;
        }
        /** Otherwise checks if the commmand is complete */
        else {
            return false;
        }
    }
}
