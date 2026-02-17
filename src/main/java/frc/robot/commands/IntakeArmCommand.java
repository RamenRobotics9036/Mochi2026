package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the intake arm motor to raise/lower the intake arm
 */
public class IntakeArmCommand extends Command {
    // Subsystem and controller references
    private final IntakeSubsystem m_intake;
    private final CommandXboxController m_controller;

    /**
     * Creates a new IntakeArmCommand.
     * 
     * @param intake     The intake subsystem to control.
     * @param controller The Xbox controller used to provide haptic (rumble) feedback.
     */
    public IntakeArmCommand(IntakeSubsystem intake, CommandXboxController controller) {
        m_intake = intake;
        m_controller = controller;
        
        // Ensure no other intake commands run simultaneously
        addRequirements(m_intake);
    }

    /**
     * Resets the internal state before starting the intake motor.
     */
    @Override
    public void initialize() {
    }

    /**
     * Periodically sets the intake motor to the constant intake speed defined in constants.
     */
    @Override
    public void execute() {
        // Right stick Y controls arm speed.
        // WPILib reports forward as negative on many controllers, so we invert it.
        double raw = -m_controller.getRightY();
        double armSpeed = MathUtil.applyDeadband(raw, DriveConstants.kJoystickDeadband);

        // Re-use intake speed constant as a safe max output scaler.
        armSpeed *= IntakeConstants.kArmSpeed;

        m_intake.setArmSpeed(armSpeed);
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
        m_intake.stop();
    }
}
