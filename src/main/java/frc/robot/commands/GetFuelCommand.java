package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/** Runs the intake rollers to collect fuel, ignoring arm movement. */
public class GetFuelCommand {
    /** Returns a Command that runs the intake rollers. */
    public static Command create(IntakeSubsystem intake) {

        return new RunCommand(() -> {
                intake.setIntakeSpeed(Constants.IntakeConstants.kIntakeSpeed);
            },
            // Dependencies:
            intake)
            .until(intake::isStalled)
            .withTimeout(5.0)
            .finallyDo(() -> {
                intake.stop();
            });
    }
}
