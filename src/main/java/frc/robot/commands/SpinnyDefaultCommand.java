package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinnyWheels;

public class SpinnyDefaultCommand extends Command{
    private SpinnyWheels m_spinny;
    private double m_spinnyDirection;
    private final Timer m_timer = new Timer();

    public SpinnyDefaultCommand(SpinnyWheels spinny){
        m_spinny = spinny;

        addRequirements(m_spinny);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        // Swaps the direction of the agitator every 3 seconds
        if (m_timer.get() >= 3.0) {
            m_spinnyDirection *= -1;
            m_timer.restart();
        }

        m_spinny.spin(m_spinnyDirection);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_spinny.stop();
    }
}