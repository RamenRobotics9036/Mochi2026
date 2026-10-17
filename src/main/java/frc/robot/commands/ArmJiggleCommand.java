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
public class ArmJiggleCommand extends Command {
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) {
        return new RunCommand(
                () -> arm.moveArmWithSpeed(-0.1),
                // Dependencies:
                arm, intake)
            .until(() -> arm.getArmAngle() <= Constants.ArmConstants.kMinArmAngle + 40)
            .andThen(
                new RunCommand(
                    () -> arm.moveArmWithSpeed(0.1)
                )
                .until(() -> arm.getArmAngle() >= Constants.ArmConstants.kMaxArmAngle - 40)
            )
            .repeatedly();
    }
}