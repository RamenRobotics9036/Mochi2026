package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUpCommand extends Command{
    private ClimberSubsystem m_climber;
    private Timer m_timer;

    public ClimbUpCommand(ClimberSubsystem climber){
        m_climber = climber;
        m_timer = new Timer();

        addRequirements(m_climber);
    }
    
    @Override
    public void initialize(){
        m_timer.start();
    }

    @Override
    public void execute() {
        m_climber.setClimbSpeed(Constants.ClimberConstants.kClimbUpSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
        System.out.println("     Time to raise:" + m_timer.get());
    }
}
