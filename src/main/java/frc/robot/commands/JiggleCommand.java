package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class JiggleCommand extends Command {
    private final CommandSwerveDrivetrain m_swerve;
    private boolean m_isForward;
    private final Timer m_timer = new Timer();

    private final RobotCentric m_request = new RobotCentric();
    
    public JiggleCommand(CommandSwerveDrivetrain swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        System.out.println("    Jiggle initialized");
    }

    @Override
    public void execute() {
        if (m_timer.get() > DriveConstants.kSecondsToAlternate.in(Seconds)) {
            m_isForward = !m_isForward;
            m_timer.restart();
        }

    
        if (m_isForward) {
            m_swerve.setControl(m_request.withVelocityX(DriveConstants.kJiggleSpeed));
            System.out.println("    Jiggle forward");
        } else {
            m_swerve.setControl(m_request.withVelocityX(DriveConstants.kJiggleSpeed.times(-1)));
            System.out.println("    Jiggle forward");
        }
    }

    @Override
    public boolean isFinished() {
        return false; //trigger should use wileTrue
    }
}
