package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** 
 * Command to jiggle the robot's drivebase in order to help fuel enter the shooter.
 * Alternates direction on an interval determined by {@link DriveConstants.kSecondsToAlternate}.
 */
public class JiggleCommand extends Command {
    /** The subsystem for the swerve drivetrain. */
    private final CommandSwerveDrivetrain m_swerve;
    /** Whether the next direction to move should be forward. */
    private boolean m_moveForward;
    /** The timer used to alternate the direction of the jiggle. */
    private final Timer m_timer = new Timer();
    /** The control object used when commanding the drivetrain to jiggle. */
    private final RobotCentric m_request = new RobotCentric();
    
    /** 
     * Creates a command designed to jiggle the robot and keep fuel entering the shooter.
     * 
     * @param swerve The swerve drivetrain
     */
    public JiggleCommand(CommandSwerveDrivetrain swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    /** Runs when the command is initialized. Resets the timer for jiggling. */
    @Override
    public void initialize() {
        m_timer.restart();
        System.out.println("    Jiggle initialized");
    }

    /** 
     * The periodic function for the command. Handles direction changing and the actual commanding
     * of the swerve drivetrain.
     */
    @Override
    public void execute() {
        if (m_timer.get() > DriveConstants.kSecondsToAlternate.in(Seconds)) {
            m_moveForward = !m_moveForward;
            m_timer.restart();
        }

    
        if (m_moveForward) {
            m_swerve.setControl(m_request.withVelocityX(DriveConstants.kJiggleSpeed));
            System.out.println("    Jiggle forward");
        } else {
            m_swerve.setControl(m_request.withVelocityX(DriveConstants.kJiggleSpeed.times(-1)));
            System.out.println("    Jiggle backward");
        }
    }

    /** Relinquish control of the drivetrain when the command is interrupted */
    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_swerve.setControl(m_request.withVelocityX(0).withVelocityY(0));
    }

    /** Command doesn't end naturally and must be interrupted. */
    @Override
    public boolean isFinished() {
        return false; //trigger should use whileTrue
    }
}