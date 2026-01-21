package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSystem;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class ElevatorToPositionCommand extends Command {
    private final ElevatorSystem m_elevator;
    private final double m_targetPosition;

    public ElevatorToPositionCommand(ElevatorSystem elevator, double targetPosition) {
        m_elevator = elevator;
        m_targetPosition = targetPosition;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        // Set desire position once
        m_elevator.setPosition(m_targetPosition);
    }

    @Override
    public void execute() {
        // Repeatedly set it to ensure it holds?
        // setPosition updates the PID reference.
        // If we want to ensure we stay there against other commands trying to change it, we must be the active command.
        m_elevator.setPosition(m_targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Finish when within tolerance
        return Math.abs(m_elevator.getPosition() - m_targetPosition) < ElevatorConstants.tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // If interrupted, we do nothing (SparkMax holds last position if in brake mode?)
        // Or should we stop?
        // ElevatorSystem.stopSystem() stops the motor (0 output).
        // If we stop, gravity might pull it down?
        // The default command will take over.
        // If default command holds last setpoint, and we interrupted, the default command will see the setpoint we set (via getDesiredPosition) and hold it.
        // So we don't need to do anything here.
        // Assuming default command uses getDesiredPosition().
    }
}
