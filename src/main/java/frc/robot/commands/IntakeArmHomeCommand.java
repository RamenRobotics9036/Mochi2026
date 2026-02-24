package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Runs the intake arm homing sequence */
public class IntakeArmHomeCommand {
    /** Returns a Command that begins the intake arm homing sequence (fires once on press). */
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) {
        return new InstantCommand(
                arm::beginHoming,
                // Dependencies:
                arm, intake);
    }
}
