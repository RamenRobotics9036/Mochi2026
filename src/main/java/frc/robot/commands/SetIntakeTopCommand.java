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
                () -> arm.setArmPosition(Constants.ArmConstants.kMaxArmAngle),
                // Dependencies:
                arm, intake)
            // Initialize the homing sequence before the command starts running updates
            .beforeStarting(arm::beginHoming)
            .withTimeout(2.0)
            .finallyDo(arm::stop);
    }
}
