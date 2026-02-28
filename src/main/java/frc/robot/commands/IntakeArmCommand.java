package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Runs the arm arm motor to raise/lower the arm arm
 */
public class IntakeArmCommand extends Command {
    // Subsystem and controller references
    private final ArmSubsystem m_arm;
    private final CommandXboxController m_controller;

    /**
     * Creates a new armArmCommand.
     *
     * @param arm     The arm subsystem to control.
     * @param controller The Xbox controller used to provide haptic (rumble) feedback.
     */
    public IntakeArmCommand(ArmSubsystem arm, CommandXboxController controller) {
        m_arm = arm;
        m_controller = controller;

        // Ensure no other arm commands run simultaneously
        addRequirements(m_arm);
    }

    /**
     * Resets the internal state before starting the arm motor.
     */
    @Override
    public void initialize() {
    }

    /**
     * Periodically sets the arm motor to the constant arm speed defined in constants.
     */
    @Override
    public void execute() {
        /*// Right stick Y controls arm speed.
        // WPILib reports forward as negative on many controllers, so we invert it.
        double raw = -m_controller.getRightY();
        double armSpeed = MathUtil.applyDeadband(raw, DriveConstants.kJoystickDeadband);

        // Re-use arm speed constant as a safe max output scaler.
        armSpeed *= ArmConstants.kArmSpeed;

        m_arm.moveArmWithSpeed(armSpeed);*/

        if (m_controller.rightTrigger().getAsBoolean()) {
            m_arm.moveArmWithSpeed(ArmConstants.kArmSpeed);
        } else if (m_controller.leftTrigger().getAsBoolean()) {
            m_arm.moveArmWithSpeed(-ArmConstants.kArmSpeed);
        } else {
            m_arm.stop();
        }
    }

    /**
     * Detection logic for game piece acquisition.
     *
     * @return true if the motor current exceeds the stall threshold, signaling the piece is inside.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Handles cleanup when the command ends.
     * Stops the motor and triggers the rumble if the piece was successfully acquired.
     *
     * @param interrupted true if the command was canceled by the driver or another command.
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the arm motor
        m_arm.stop();
    }
}
