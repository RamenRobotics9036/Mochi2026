package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Deploys the intake arm and runs the intake rollers to collect fuel. */
public class GetFuelCommand {
    /** Returns a Command that deploys the arm and runs the intake. */
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) {
        return new RunCommand(() -> {
                arm.setArmPosition(Constants.ArmConstants.kMaxArmAngle / Constants.ArmConstants.kArmGearRatio);
                intake.setIntakeSpeed(Constants.IntakeConstants.kIntakeSpeed);
            },
            // Dependencies:
            arm, intake)
            .withTimeout(Constants.AutoConstants.k_getFuelDuration)
            .finallyDo(() -> {
                arm.stop();
                intake.stop();
            });
    }
}
