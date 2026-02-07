package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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

    /**
     * Creates a new VisonDefaultCommand.
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
        // No initialization required; the subsystem maintains its current state
    }

    @Override
    public void execute() {
        boolean TagVisible = (LimelightHelpers.getFiducialID(m_limelightName) != -1.0);
        System.out.println(TagVisible);
    }

    /**
     * This command currently isn't complete so it doesn't
     * have a finished state.
     * It stops if another request is applied.
     */
    @Override
    public boolean isFinished() {
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
        /** Otherwise continues the command, as there is no finished
         * state for this unfinished command */
        else {
            return false;
        }
    }

    // /**
    //  * Stops elevator movement when the command is interrupted or cleared.
    //  * 
    //  * @param interrupted whether the command was interrupted/canceled
    //  */
    // @Override
    // public void end(boolean interrupted) {
    //     // Stop the elevator motor to prevent unintended movement
    //     //m_elevator.stopSystem();
    // }
}
