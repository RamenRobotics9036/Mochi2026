package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class JiggleCommand extends Command {
    private final CommandSwerveDrivetrain m_swerve;
    private final ArmSubsystem m_arm;
    private boolean m_isForward;
    private final Timer m_timer = new Timer();

    private final RobotCentric m_request = new RobotCentric();
    
    public JiggleCommand(CommandSwerveDrivetrain swerve, ArmSubsystem arm) {
        m_swerve = swerve;
        m_arm = arm;
        addRequirements(m_swerve, m_arm);
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

        double wigglePower = (m_timer.get() < (DriveConstants.kSecondsToAlternate.in(Seconds) / 2.0)) 
            ? ArmConstants.kArmSpeed * 0.5 
            : -ArmConstants.kArmSpeed * 0.5;
        
        m_arm.moveArmWithSpeed(wigglePower);
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.setControl(m_request.withVelocityX(0).withVelocityY(0));
        m_arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false; //trigger should use wileTrue
    }
}
