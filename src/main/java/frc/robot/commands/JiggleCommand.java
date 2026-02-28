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
    private enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT
    }

    private final CommandSwerveDrivetrain m_swerve;
    private Direction m_direction = Direction.FORWARD;
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
        if (m_timer.get() <= DriveConstants.kSecondsToAlternate.in(Seconds)) {
            switch (m_direction) {
                case FORWARD:
                    m_direction = Direction.BACKWARD;
                    System.out.println("    Going backward");
                    break;
                case BACKWARD:
                    m_direction = Direction.LEFT;
                    System.out.println("    Going left");
                    break;
                case LEFT:
                    m_direction = Direction.RIGHT;
                    System.out.println("    Going right");
                    break;
                case RIGHT:
                    m_direction = Direction.FORWARD;
                    System.out.println("    Going forward");
                    break;
            }

            m_timer.reset();
        }

    
        switch (m_direction) {
            case FORWARD:
                m_swerve.setControl(m_request.withVelocityX(DriveConstants.kJiggleSpeed));;
                break;
            case BACKWARD:
                m_swerve.setControl(m_request.withVelocityX(DriveConstants.kJiggleSpeed.times(-1)));
                break;
            case LEFT:
                m_swerve.setControl(m_request.withVelocityY(DriveConstants.kJiggleSpeed));
                break;
            case RIGHT:
                m_swerve.setControl(m_request.withVelocityY(DriveConstants.kJiggleSpeed.times(-1)));
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false; //trigger should use wileTrue
    }
}
