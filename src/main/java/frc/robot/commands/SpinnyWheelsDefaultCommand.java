package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinnyWheels;

public class SpinnyWheelsDefaultCommand extends Command{
    private final SpinnyWheels m_spinnyWheels;
    private final Timer m_timer = new Timer();

    public SpinnyWheelsDefaultCommand(SpinnyWheels spinnyWheels){
        m_spinnyWheels = spinnyWheels;
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        if(m_timer.get()%4 == 0){ //TODO: replace 4 with a constant
            m_spinnyWheels.spinBack();
        } else {
            m_spinnyWheels.spin();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_spinnyWheels.stop();
    }
}
