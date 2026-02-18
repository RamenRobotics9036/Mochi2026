package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmToPoseCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final double m_targetAngle;
    private final Timer m_timer = new Timer();
    
    public ArmToPoseCommand(IntakeSubsystem intake, double targetAngle) {
        m_intake = intake;
        m_targetAngle = targetAngle;

        addRequirements(m_intake);
    }

        @Override
        public void initialize() {
            m_timer.reset();
            m_timer.start();

            m_intake.setArmPosition(m_targetAngle);
        }
    
        @Override
        public void execute() {
        }

        @Override
        public boolean isFinished(){
        if (m_timer.get() > IntakeConstants.kMaxTime) {
            System.out.println("WARNING: IntakeToPoseCommand timed out!");
            return true;
        }
        return MathUtil.applyDeadband(m_targetAngle - m_intake.getPosition(), IntakeConstants.kTolerance) == 0;
    }
}