package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    private final Debouncer m_debouncer = new Debouncer(0.25, DebounceType.kRising);
    private double indexerDirection = 0.0;
    private boolean indexerEnabled = false;

    public ShooterDefaultCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, CommandXboxController controller){
        m_shooter = shooter;
        m_indexer = indexer;
        m_controller = controller;

        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize() {
        m_debouncer.calculate(false); //In case the default command gets interrupted
    }

    @Override
    public void execute() {
        if (m_controller.y().getAsBoolean()) indexerDirection = 1.0;
        else if (m_controller.rightBumper().getAsBoolean()) indexerDirection = -1.0;
        else indexerDirection = 0.0;

        indexerEnabled = m_debouncer.calculate(indexerDirection != 0.0);

        if (indexerDirection != 0.0) {
            m_shooter.setSpeed(ShooterConstants.kShootSpeed);
            
            if (indexerEnabled) {
                m_indexer.setSpeed(indexerDirection * IndexerConstants.kIndexSpeed);
            }

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