package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Runs the intake motor to collect a game piece and provides haptic feedback upon success.
 * 
 * <p>The command monitors the motor's current/stall status. Once a stall is detected 
 * (suggesting the piece is secured), the command finishes and triggers a rumble on 
 * the driver's controller.
 */
public class IntakeDefaultCommand extends Command {
    // Subsystem and controller references
    private final IntakeSubsystem m_intake;
    private final CommandXboxController m_controller;

    /**
     * Creates a new IntakeCommand.
     * 
     * @param intake     The intake subsystem to control.
     * @param controller The Xbox controller used to provide haptic (rumble) feedback.
     */
    public IntakeDefaultCommand(IntakeSubsystem intake, CommandXboxController controller) {
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
        // Run the intake motor at the predefined speed
        if (m_controller.a().getAsBoolean()) {
            m_intake.setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        };
    }

    /**
     * Since this is a default command, always returns false.
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
        // Stop the intake motor
        m_intake.stop();
    }
}
