package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Lowers the intake arm to the deployed (bottom) position. */
public class SetIntakeBottomCommand {
    /** Returns a Command that drives the arm down until it reports deployed. */
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) {
        // Use runOnce so the PID starts and keeps holding the position
        return Commands.runOnce(() -> arm.setArmPosition(Constants.ArmConstants.kMaxArmAngle), arm)
            // Wait until the arm reports it is at the bottom
            .until(arm::isArmDeployed)
            .withTimeout(3.5);
            // Note: removed finallyDo(stop) so the motor holds the arm down
    }
}
