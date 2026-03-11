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

    /**
     * Gets controller input and handles the shooter and indexer accordingly.
     * Designed to be set as the default command for the {@link ShooterSubsystem}
     * and the {@link IndexerSubsystem}.
     * 
     * @param shooter
     * @param indexer
     * @param controller
     */
    public ShooterDefaultCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, CommandXboxController controller){
        /** The subsystem for the motors that shoot fuel into the Alliance Hub. */
        m_shooter = shooter;
        /** The subsystem for the motors that feed fuel into the shooter. */
        m_indexer = indexer;
        /** The operator's Xbox controller. */
        m_controller = controller;

        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize() {
        m_debouncer.calculate(false); //In case the default command gets interrupted
    }

    /** 
     * Spins up the shooter for a period of time defined by {@link IndexerConstants.kIndexDelay},
     * then activates the indexer to feed it fuel.
     * 
     * Right bumper makes the indexer spin backwards in case balls get stuck.
     */
    @Override
    public void execute() {
        // Check controller input to determine whether to move the indexer forward,
        // backward, or not at all. In all but the last case, the shooter should
        // be commanded to spin as well.
        if (m_controller.y().getAsBoolean()) indexerDirection = 1.0;
        else if (m_controller.rightBumper().getAsBoolean()) indexerDirection = -1.0;
        else indexerDirection = 0.0;

        // Determines whether to actually spin the indexer.
        // Is false for a short delay so the shooter can spin up.
        indexerEnabled = m_debouncer.calculate(indexerDirection != 0.0);

        // So long as the indexer is theoretically meant to be running,
        // the shooter is as well, regardless of whether or not the
        // indexer is on delay.
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

    /** 
     * Default commands are not designed to end on their own.
     * Must be interrupted.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Stops the shooter and indexer if the command is interrupted. */
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_indexer.stop();
    }
}