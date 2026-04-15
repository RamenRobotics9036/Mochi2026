package frc.robot.commands; 
 
import edu.wpi.first.wpilibj2.command.Command; 
import edu.wpi.first.wpilibj2.command.RunCommand; 
import frc.robot.Constants; 
import frc.robot.subsystems.ArmSubsystem; 
import frc.robot.subsystems.IntakeSubsystem; 
 
/** Raises the intake arm to the stowed (top) position. */ 
public class SetIntakeTopCommand { 
    /** Returns a Command that drives the arm up, stopping it when done or interrupted. */ 
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) { 
        return new RunCommand( 
                () -> {
                    if (!arm.isArmHomed()) {
                        return;
                    }

                    if (arm.getArmPosition() > Constants.ArmConstants.kMinArmAngle + 1.0) {
                        arm.moveArmWithSpeed(-Constants.ArmConstants.kArmSpeed);
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
            .until(() -> arm.isArmHomed() && arm.getArmPosition() <= Constants.ArmConstants.kMinArmAngle + 1.0)
            .withTimeout(Constants.AutoConstants.k_intakeTopDuration) 
            .finallyDo(arm::stop); 
    } 
} 
