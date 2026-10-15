package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Lowers the intake arm to the deployed (bottom) position. */
public class SetIntakeBottomCommand {
    /** Returns a Command that drives the arm down until it reports deployed. */
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) {
        // Note: kMaxArmAngle represents the deployed position, since the
        // arm starts in the up position and therefore up is 0
        return new RunCommand(
                // kMaxArmAngle represents the lowest position, as the intake begins up
                () -> arm.setArmPosition(Constants.ArmConstants.kMaxArmAngle),
                // Dependencies:
                arm, intake)
            .until(arm::isArmDeployed)
            .withTimeout(Constants.AutoConstants.k_intakeBottomDuration)
            .finallyDo(arm::stop);
    }
}
