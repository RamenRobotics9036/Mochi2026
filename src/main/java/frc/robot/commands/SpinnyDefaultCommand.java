package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpinnyWheelsConstants;
import frc.robot.subsystems.SpinnyWheels;

public class SpinnyDefaultCommand extends Command{
    /** The subsystem for the agitator (aka "spinny" or the "Christmas tree"). */
    private SpinnyWheels m_spinny;
    private final Timer m_timer = new Timer();

    /**
     * Alternates the direction of the agitator in order to better agitate the fuel.
     * Designed to be set as the default command for the {@link SpinnyWheels} subsystem.
     * 
     * @param spinny The subsystem for the agitator (aka "spinny" or the "Christmas tree").
     */
    public SpinnyDefaultCommand(SpinnyWheels spinny){
        m_spinny = spinny;

        addRequirements(m_spinny);
    }

    /** Resets the timer on the agitator when the command is initialized. */
    @Override
    public void initialize() {
        m_timer.restart();
    }

    /** Alternates the direction of the agitator to more effectively stir up the fuel. */
    @Override
    public void execute() {
        //Uses mod to cut into sections of kResetTime seconds. For the first kClockwiseTime seconds of each, runs ckockwise.
        //TODO: is there a reason we're using modulo rather than simply resetting the timer when the direction changes?
        if(m_timer.get() % SpinnyWheelsConstants.kTotalTime.in(Seconds) < SpinnyWheelsConstants.kClockwiseTime.in(Seconds)){
            m_spinny.spinClockwise();
        } else {
            m_spinny.spinCounterclockwise();
        }
    }

    /** 
     * Default commands are not designed to end on their own.
     * Must be interrupted.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Stops the agitator when the command ends. */
    @Override
    public void end(boolean interrupted) {
        m_spinny.stop();
    }
}