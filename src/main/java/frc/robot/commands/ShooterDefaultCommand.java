package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterDefaultCommand extends Command{
    private ShooterSubsystem m_shooter;
    private IndexerSubsystem m_indexer;
    private CommandXboxController m_controller;

    public ShooterDefaultCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, CommandXboxController controller){
        m_shooter = shooter;
        m_indexer = indexer;
        m_controller = controller;

        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (m_controller.y().getAsBoolean()){
            m_shooter.setSpeed(ShooterConstants.kShootSpeed);
            m_indexer.setSpeed(IndexerConstants.kIndexSpeed);
        } else if (m_controller.rightBumper().getAsBoolean()) {
            m_shooter.setSpeed(ShooterConstants.kShootSpeed);
            m_indexer.setSpeed(-IndexerConstants.kIndexSpeed);
        } else {
            m_shooter.stop();
            m_indexer.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_indexer.stop();
    }
}