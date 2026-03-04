package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

/** Runs the intake arm homing sequence */
public class IntakeArmHomeCommand {
    /** Returns a Command that begins the intake arm homing sequence (fires once on press). */
    public static Command create(ArmSubsystem arm) {
        return new InstantCommand(arm::beginHoming, arm);
    }
}
