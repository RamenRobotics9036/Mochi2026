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
    /** The subsystem for the motors used to shoot the ball */
    private ShooterSubsystem m_shooter;
    /** The subsystem for the indexer used to feed the shooter */
    private IndexerSubsystem m_indexer;
    /** The operator controller, used to control the shooter and indexer */
    private CommandXboxController m_controller;
    /** The Debouncer object used to delay indexer activation */
    private final Debouncer m_debouncer = new Debouncer(IndexerConstants.kIndexDelay, DebounceType.kRising);
    /** The direction the indexer is spinning (from -1.0 to 1.0) */
    private double indexerDirection = 0.0;
    /** Whether the indexer should be running */
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

    /** 
     * Spins up the shooter for 0.25 seconds, then activates the indexer to feed it.
     * 
     * Right bumper makes the indexer spin backwards in case balls get stuck.
     */
    @Override
    public void execute() {
        if (m_controller.y().getAsBoolean()) indexerDirection = 1.0;
        else if (m_controller.rightBumper().getAsBoolean()) indexerDirection = -1.0;
        else indexerDirection = 0.0;

        indexerEnabled = m_debouncer.calculate(indexerDirection != 0.0);

        if (indexerDirection != 0.0) {
            m_shooter.setSpeed(ShooterConstants.kShootSpeed);
            
            if (indexerEnabled) {
                m_indexer.setSpeed(Math.signum(indexerDirection) * IndexerConstants.kIndexSpeed);
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