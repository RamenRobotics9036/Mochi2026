package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
    
    public JiggleCommand(CommandSwerveDrivetrain swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        if (m_timer.get() <= 0.5) { //TODO: filler value
            switch (m_direction) {
                case FORWARD:
                    m_direction = Direction.BACKWARD;
                    break;
                case BACKWARD:
                    m_direction = Direction.LEFT;
                    break;
                case LEFT:
                    m_direction = Direction.RIGHT;
                    break;
                case RIGHT:
                    m_direction = Direction.FORWARD;
                    break;
            }

            m_timer.reset();
        }

    
        switch (m_direction) {
            case FORWARD:
                m_swerve.applyRequest(() -> new RobotCentric().withVelocityX(DriveConstants.kJiggleSpeed));
                break;
            case BACKWARD:
                m_swerve.applyRequest(() -> new RobotCentric().withVelocityX(DriveConstants.kJiggleSpeed.times(-1)));
                break;
            case LEFT:
                m_swerve.applyRequest(() -> new RobotCentric().withVelocityY(DriveConstants.kJiggleSpeed));
                break;
            case RIGHT:
                m_swerve.applyRequest(() -> new RobotCentric().withVelocityY(DriveConstants.kJiggleSpeed.times(-1)));
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return false; //trigger should use wileTrue
    }
}
