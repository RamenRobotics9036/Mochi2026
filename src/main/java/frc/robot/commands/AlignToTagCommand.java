package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.visutils.SingleCamOdometry;
import frc.robot.Constants.VisionConstants;
import frc.robot.botconfig.BotConfigInterface;
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
    private final BotConfigInterface m_configInterface;

    private CommandSwerveDrivetrain m_drivetrain;
    private String m_limelightName;
    private CommandXboxController m_joystick;
    private Rotation2d m_targetRotation;
    private SwerveRequest.RobotCentric m_driveRequest;
    private boolean alignmentFailed = false;
    private double m_targetRotationDegrees;
    private final Timer m_timer = new Timer();
    private PIDController pid = new PIDController(0.1, 0, 0);
    private double m_maxAngularVelocity;

    /**
     * Creates a new AlignToTagCommand.
     *
     * @param drivetrain The drivetrain subsystem to be supplied
     * @param joystick The Xbox controller to use for interruption
     */
    public AlignToTagCommand(
        BotConfigInterface configInterface,
        CommandSwerveDrivetrain drivetrain,
        CommandXboxController joystick,
        double MaxAngularVelocity) {

        m_configInterface = configInterface;

        m_drivetrain = drivetrain;
        m_joystick = joystick;
        m_maxAngularVelocity = MaxAngularVelocity;

        /** The limelight's name is different in simulation. */
        if (Robot.isSimulation()) {
            m_limelightName = VisionConstants.kLimelightNameSim;
        }
        else {
            m_limelightName = m_configInterface.getVisionLimelightNameReal();
        }

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        /** Ensures that the PID system understands that
         * its motion is circular */
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(m_configInterface.getAlignmentErrorMargin());

        /** Resets the command */
        alignmentFailed = false;
        m_timer.restart();
        m_driveRequest = new SwerveRequest.RobotCentric();

        /** Ensures the limelight is even locked on to any AprilTags to align to */
        if (LimelightHelpers.getFiducialID(m_limelightName) == -1.0) {
            alignmentFailed = true;
            return;
        }

        /** If so, calculate the desired rotation */
        double targetRotationDegrees = -1 * LimelightHelpers.getTX(m_limelightName);
        SwerveDriveState driveState = m_drivetrain.getState();
        Pose2d driveStatePose = driveState.Pose;

        /** Gets the desired rotation using the relative
         * rotation needed and the current Rotation2D */
        m_targetRotation = Rotation2d.fromDegrees (
            driveStatePose.getRotation().getDegrees()
            + targetRotationDegrees
        );

        m_targetRotationDegrees = m_targetRotation.getDegrees();
        System.out.println("Attempting to align to AprilTag.");
    }

    @Override
    public void execute() {
        /** Annoyingly enough, execute will run once before
         * isFinished can check to see if it's done, so this
         * counteracts that and ensures the robot doesn't
         * randomly turn slightly. */
        if (alignmentFailed) {
            return;
        }

        SwerveDriveState driveState = m_drivetrain.getState();
        Pose2d driveStatePose = driveState.Pose;
        double rotationOffset = driveStatePose.getRotation().getDegrees() - m_targetRotationDegrees;

        m_drivetrain.setControl(m_driveRequest.withRotationalRate(
            /** Ensures that the angular velocity doesn't exceed
             * the maximum angular speed. */
            MathUtil.clamp(
                pid.calculate(rotationOffset),
                -m_maxAngularVelocity,
                m_maxAngularVelocity
                )
            ));
    }

    /**
     * This command currently isn't complete so it doesn't
     * have a finished state.
     * It stops if another request is applied.
     */
    @Override
    public boolean isFinished() {
        /** Safety exit after 15 seconds */
        if (m_timer.get() > 15.0) {
            System.out.println("Alignment command timed out.");
            return true;
        }

        /** If the robot isn't locked onto any AprilTags,
         * the command fails rather than using a default
         * position. */
        if (alignmentFailed) {
            System.out.println("WARNING: Failed to align to AprilTag!");
            return true;
        }

        /** Checks to see if the command is done */
        if (pid.atSetpoint()) {
            System.out.println("Alignment complete!");
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
        if (absoluteMoveInput > m_configInterface.getSwerveMoveInterruptionSensitivity()) {
            System.out.println("Command interrupted.");
            return true;
        }
        /** Stops if the user tries to manually turn the robot */
        else if (absoluteTurnInput > m_configInterface.getSwerveTurnInterruptionSensitivity()) {
            System.out.println("Command interrupted.");
            return true;
        }
        /** Otherwise, keep going! */
        else {
            return false;
        }
    }
}
