package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSystem;

/**
 * A command that moves the elevator to a specific setpoint.
 * This is typically used for automated scoring positions (e.g., Intake, Low Goal, High Goal).
 * The command finishes once the elevator is within the defined tolerance of the target.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class ElevatorToPositionCommand extends Command {
    private final ElevatorSystem m_elevator;
    private final double m_targetPosition;

    /**
     * Creates a new ElevatorToPositionCommand.
     *
     * @param elevator       The elevator subsystem.
     * @param targetPosition The desired height/encoder count to reach.
     */
    public ElevatorToPositionCommand(ElevatorSystem elevator, double targetPosition) {
        m_elevator = elevator;
        m_targetPosition = targetPosition;
        
        // Block other elevator commands (like manual control) while this is running
        addRequirements(m_elevator);
    }

    /**
     * Called when the command is initially scheduled.
     * Sends the target position to the subsystem's PID controller.
     */
    @Override
    public void initialize() {
        // Set the elevator to the desired target position
        m_elevator.setPosition(m_targetPosition);
    }

    /**
     * Called repeatedly while the command is scheduled.
     * Continues to update the PID reference to ensure the elevator moves toward 
     * or holds the target position against external forces like gravity.
     */
    @Override
    public void execute() {
        // Continuously reinforce the target position
        m_elevator.setPosition(m_targetPosition);
    }

    /**
     * Returns true when the elevator's actual position is within the acceptable 
     * deadband/tolerance defined in constants.
     */
    @Override
    public boolean isFinished() {
        // Check if the elevator is within the tolerance of the target position
        return Math.abs(m_elevator.getPosition() - m_targetPosition) < ElevatorConstants.tolerance;
    }

    /**
     * Called when the command ends or is interrupted.
     * 
     * Note: We do not call stopSystem() here because the ElevatorDefaultCommand 
     * will immediately take over and use getDesiredPosition() to maintain this 
     * new height. Stopping would cause the elevator to drop under gravity.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        // No action needed; hand-off to default command preserves the setpoint.
    }
}
