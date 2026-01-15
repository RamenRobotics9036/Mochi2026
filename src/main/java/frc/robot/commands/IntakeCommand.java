package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * IntakeCommand runs the intake motor until a stall is detected (indicating a successful intake)
 * or until the command is interrupted.
 */
public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;

    public IntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setSpeed(IntakeConstants.kIntakeSpeed);
    }

    @Override
    public boolean isFinished() {
        // Stop automatically if we sense a stall (ball/piece acquired)
        return m_intake.isStalled();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
