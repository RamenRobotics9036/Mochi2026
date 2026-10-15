package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** 
 * Command to jiggle the robot's drivebase in order to help fuel enter the shooter.
 * Alternates direction on an interval determined by {@link DriveConstants.kSecondsToAlternate}.
 */
public class JiggleCommand extends Command {
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) {
        return new RunCommand(
                () -> arm.setArmPosition(Constants.ArmConstants.kMinArmAngle),
                // Dependencies:
                arm, intake)
            .until(() -> arm.isArmRetracted())
            .andThen(
                new RunCommand(
                    () -> arm.setArmPosition(Constants.ArmConstants.kMaxArmAngle)
                )
                .until(() -> arm.isArmDeployed())
            )
            .repeatedly();
    }
}