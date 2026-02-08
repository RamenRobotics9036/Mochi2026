package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

/**FOR TESTING PURPOSES ONLY. Uses joystick input to manually change the flywheel speed.*/
public class ShooterTestCommand extends Command {
    private final ShooterSubsystem m_shooter;

    private CommandXboxController m_controller;

    public ShooterTestCommand(ShooterSubsystem shooter, CommandXboxController controller) {
        m_shooter = shooter;
        m_controller = controller;
        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //May want to chaneg input for testing different things.
        m_shooter.setSpeed(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1)*0.1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
