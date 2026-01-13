package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorDefaultCommandConstants;
import frc.robot.subsystems.ElevatorSystem;
import java.util.function.DoubleSupplier;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class ElevatorDefaultCommand extends Command{
    private ElevatorSystem m_elevator;
    private DoubleSupplier m_joystick;

    public ElevatorDefaultCommand(ElevatorSystem elevator, DoubleSupplier joystick){
        m_elevator = elevator;
        m_joystick = joystick;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Update setpoint based on joystick input.
        // We use getDesiredPosition() (the last setpoint) instead of getPosition()
        // to avoid drift caused by gravity or lag when joystick is 0.
        // If joystick is 0, we simply maintain the last setpoint.
        
        double currentSetpoint = m_elevator.getDesiredPosition();
        double change = m_joystick.getAsDouble() * ElevatorDefaultCommandConstants.kElevatorSpeed;
        
        m_elevator.setPosition(
            MathUtil.clamp(currentSetpoint + change,
            ElevatorConstants.kMaxElevatorPosition, // min (most negative)
            ElevatorConstants.kDownElevatorPosition)); // max (0 or positive)
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stopSystem();
    }
}
