package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorDefaultCommandConstants;
import frc.robot.subsystems.ElevatorSystem;
import java.util.function.DoubleSupplier;

/**
 * Standard command for manual elevator control.
 * This command continuously updates the elevator's target position based on joystick input.
 * It is designed to be set as the default command for the {@link ElevatorSystem}.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class ElevatorDefaultCommand extends Command {
    private ElevatorSystem m_elevator;
    private DoubleSupplier m_joystick;

    /**
     * Creates a new ElevatorDefaultCommand.
     *
     * @param elevator The elevator subsystem to be controlled.
     * @param joystick A supplier for the joystick axis value (typically -1.0 to 1.0).
     */
    public ElevatorDefaultCommand(ElevatorSystem elevator, DoubleSupplier joystick) {
        m_elevator = elevator;
        m_joystick = joystick;

        // Ensure no other command uses the elevator while this is running
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        // No initialization required; the subsystem maintains its current state
    }

    /**
     * Periodically updates the elevator setpoint.
     * 
     * Calculates the change based on joystick input and applies it to the previous 
     * desired position. This prevents "setpoint drifting" that occurs when relying 
     * on the current physical position of a motor under load.
     */
    @Override
    public void execute() {
        // Retrieve the last commanded setpoint to calculate the next incremental move
        double currentSetpoint = m_elevator.getDesiredPosition();
        
        // Scale the joystick input by a constant speed factor
        double change = m_joystick.getAsDouble() * ElevatorDefaultCommandConstants.kElevatorSpeed;
        
        // Apply the change and clamp the result within soft limits to prevent mechanical damage
        m_elevator.setPosition(
            MathUtil.clamp(
                currentSetpoint + change,
                ElevatorConstants.kMaxElevatorPosition, // Minimum value (Lower limit)
                ElevatorConstants.kDownElevatorPosition  // Maximum value (Upper limit)
            )
        );
    }

    /**
     * This command never ends naturally as it is a default command 
     * meant to provide constant manual control.
     */
    @Override
    public boolean isFinished() {
        // Default command runs indefinitely
        return false;
    }

    /**
     * Stops elevator movement when the command is interrupted or cleared.
     * 
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the elevator motor to prevent unintended movement
        m_elevator.stopSystem();
    }
}
