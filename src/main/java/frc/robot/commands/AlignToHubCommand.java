package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import java.lang.Math;
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
import frc.robot.Constants.VisionConstants;
import frc.robot.botconfig.BotConfigInterface;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import java.util.function.DoubleSupplier;

/** Aligns the robot to the faced AprilTag. */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class AlignToHubCommand extends Command {
    /** Information about the current robot. */
    private final BotConfigInterface m_configInterface;

    /** The subsystem for the swerve drivetrain. */
    private CommandSwerveDrivetrain m_drivetrain;
    /** The driver's Xbox controller. */
    private CommandXboxController m_joystick;
    /** The desired rotation of the robot. */
    private Rotation2d m_targetRotation;
    /** The control object to use when commanding the robot to turn. */
    private SwerveRequest.FieldCentric m_driveRequest;
    /** Is true if no AprilTags were within sight when the command was called. */
    private boolean alignmentFailed = false;
    /** The timer representing how long the command has been running. */
    private final Timer m_timer = new Timer();
    /** The PID controller used to control the alignment speed. */
    private PIDController pid = new PIDController(0.1, 0, 0);
    /** The limit on angular velocity for rotation. */
    private double m_maxAngularVelocity;

    /**
     * Creates a new AlignToTagCommand.
     *
     * @param drivetrain The drivetrain subsystem to be supplied
     * @param joystick The Xbox controller to use for interruption
     */
    public AlignToHubCommand(
        BotConfigInterface configInterface,
        CommandSwerveDrivetrain drivetrain,
        CommandXboxController joystick,
        double MaxAngularVelocity) {

        m_configInterface = configInterface;

        m_drivetrain = drivetrain;
        m_joystick = joystick;
        m_maxAngularVelocity = MaxAngularVelocity;
        
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        // Ensures that the PID system understands that its motion is circular.
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(m_configInterface.getAlignmentErrorMargin());

        // Resets the command.
        alignmentFailed = false;
        m_timer.restart();
        m_driveRequest = new SwerveRequest.FieldCentric();

        System.out.println("Attempting to align to AprilTag.");
    }

    /**
     * The periodic function for the alignment command.
     * Handles the actual rotation and rotation speed.
     */
    @Override
    public void execute() {
        // Annoyingly enough, execute will run once before
        // isFinished can check to see if it's done, so this
        // counteracts that and ensures the robot doesn't
        // randomly turn slightly.
        if (alignmentFailed) {
            return;
        }
    
        // Gets the desired rotation using the atan of the robot and hub
        m_targetRotation = Rotation2d.fromRadians(Math.atan(m_drivetrain.getState().Pose.getX()/m_drivetrain.getState().Pose.getX()));

        // Checks to see how far off the robot is from its rotation target
        SwerveDriveState driveState = m_drivetrain.getState();
        Pose2d driveStatePose = driveState.Pose;
        double remainingRotation = driveStatePose.getRotation().getRadians() - m_targetRotation.getRadians();

        m_drivetrain.setControl(m_driveRequest.withRotationalRate(
            // Ensures that the angular velocity doesn't exceed the maximum angular speed.
            MathUtil.clamp(
                pid.calculate(remainingRotation),
                -m_maxAngularVelocity,
                m_maxAngularVelocity
                )
            ));
    }

    /**
     * Stops the command if one of the following is true:
     * 
     * The command was called without any AprilTags being within view of the Limelight
     * The robot has finished alignment to the AprilTag
     * The driver is trying to move the robot and the command should be interrupted
     */
    @Override
    public boolean isFinished() {
        // Safety exit after 15 seconds.
        if (m_timer.get() > 15.0) {
            System.out.println("Alignment command timed out.");
            return true;
        }

        // If the robot isn't locked onto any AprilTags, the command
        // fails rather than using a default position.
        if (alignmentFailed) {
            System.out.println("WARNING: Failed to align to AprilTag!");
            return true;
        }

        // Checks to see if the robot has reached its target
        if (pid.atSetpoint()) {
            System.out.println("Alignment complete!");
            return true;
        }

        // Gets the absolute value of the user's left joystick input
        double absoluteMoveInput = Math.abs(
            // Gets the total value of the user's left joystick input using the Pythagorean Theorem
            Math.sqrt(
                /** x^2 + y^2 */
                Math.pow(m_joystick.getLeftX(), 2)
                + Math.pow(m_joystick.getLeftY(), 2)
            )
        );

        // Gets the absolute value of the user's right joystick input
        double absoluteTurnInput = Math.abs(
            // Gets the total value of the user's right joystick input using the Pythagorean Theorem
            Math.sqrt(
                /** x^2 + y^2 */
                Math.pow(m_joystick.getRightX(), 2)
                + Math.pow(m_joystick.getRightY(), 2)
            )
        );

        // Stops if the user tries to manually move the robot
        if (absoluteMoveInput > m_configInterface.getSwerveMoveInterruptionSensitivity()) {
            System.out.println("Command interrupted.");
            return true;
        }
        // Stops if the user tries to manually turn the robot
        else if (absoluteTurnInput > m_configInterface.getSwerveTurnInterruptionSensitivity()) {
            System.out.println("Command interrupted.");
            return true;
        }
        // Otherwise, keep going!
        else {
            return false;
        }
    }
}
