package frc.robot.commands; 
 
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj2.command.RunCommand; 
import frc.robot.Constants; 
import frc.robot.subsystems.ArmSubsystem; 
import frc.robot.subsystems.IntakeSubsystem; 
 
/** Lowers the intake arm to the deployed (bottom) position. */ 
public class SetIntakeBottomCommand { 
    /** Returns a Command that drives the arm down, stopping it when done or interrupted. */ 
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) { 
        return new RunCommand( 
                () -> {
                    if (!arm.isArmHomed()) {
                        return;
                    }

                    if (arm.getArmPosition() < Constants.ArmConstants.kMaxArmAngle - 1.0) {
                        arm.moveArmWithSpeed(Constants.ArmConstants.kArmSpeed);
                    } else {
                        arm.stop();
                    }
                }, 
                // Dependencies: 
                arm, intake) 
            // Initialize homing only if needed. If the arm is already homed, keep the current zero reference.
            .beforeStarting(() -> {
                if (!arm.isArmHomed()) {
                    arm.beginHoming();
                }
            })
            .until(() -> arm.isArmHomed() && arm.isArmDeployed())
            .withTimeout(Constants.AutoConstants.k_intakeBottomDuration) 
            .finallyDo(arm::stop); 
    } 
} 
