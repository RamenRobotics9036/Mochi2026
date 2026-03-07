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
        // $TODO - Tarun, shouldnt the IntakeBottomCommand set arm position to kMINArmAngle instead of kMAXArmAngle?
        return new RunCommand(
                () -> arm.moveArmWithSpeed(Constants.ArmConstants.kArmHomingSpeed),
                // Dependencies:
                arm, intake)
            // Initialize the homing sequence before the command starts running updates
            .beforeStarting(arm::beginHoming)
            .until(() -> Math.abs(arm.getArmPosition() - Constants.ArmConstants.kMaxArmAngle) < 2.0)
            .withTimeout(3.5)
            .finallyDo(arm::stop);
    }
}
