package frc.robot.commands; 
 
import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.wpilibj.Timer; 
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.Constants.ArmConstants; 
import frc.robot.subsystems.ArmSubsystem; 
 
public class ArmToPoseCommand extends Command { 
    private final ArmSubsystem m_arm; 
    private final double m_targetAngle; 
    private final Timer m_timer = new Timer(); 
     
    public ArmToPoseCommand(ArmSubsystem arm, double targetAngle) { 
        m_arm = arm; 
        m_targetAngle = targetAngle; 
 
        addRequirements(m_arm); 
    } 
 
        @Override 
        public void initialize() { 
            m_timer.reset(); 
            m_timer.start(); 
 
            m_arm.setArmPosition(m_targetAngle); 
        } 
     
        @Override 
        public void execute() { 
        } 
 
        @Override 
        public boolean isFinished(){ 
        if (m_timer.get() > ArmConstants.kMaxTime) { 
            System.out.println("WARNING: ArmToPoseCommand timed out!"); 
            return true; 
        } 
        return MathUtil.applyDeadband(m_targetAngle - m_arm.getPosition(), ArmConstants.kTolerance) == 0; 
    } 
}