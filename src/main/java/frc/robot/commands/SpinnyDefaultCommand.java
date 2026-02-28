package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpinnyWheelsConstants;
import frc.robot.subsystems.SpinnyWheels;

public class SpinnyDefaultCommand extends Command{
    private SpinnyWheels m_spinny;
    private double m_spinnyDirection = 1.0;
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
        //Uses mod to cut into sections of kResetTime seconds. For the first kClockwiseTime seconds of each, runs ckockwise.
        if(m_timer.get() % SpinnyWheelsConstants.kTotalTime.in(Seconds) < SpinnyWheelsConstants.kClockwiseTime.in(Seconds)){
            m_spinny.spinClockwise();
        } else {
            m_spinny.spinCounterclockwise();
        }
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