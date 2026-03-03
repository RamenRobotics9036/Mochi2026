package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Raises the intake arm to the stowed (top) position. */
public class SetIntakeTopCommand {
    /** Returns a Command that drives the arm up. */
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) {
        // Set the target to the top (stowed) position
        return Commands.runOnce(() -> arm.setArmPosition(Constants.ArmConstants.kMinArmAngle), arm)
            // Wait until the arm reports it is homed/stowed
            .until(arm::isArmHomed)
            .withTimeout(2.0);
            // Note: removed finallyDo(stop) so the motor holds the arm up
    }
}
