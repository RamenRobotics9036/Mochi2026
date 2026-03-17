package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Lowers the intake arm to the deployed (bottom) position. */
public class SetIntakeBottomCommand {
    /** Returns a Command that drives the arm down until it reports deployed. */
    public static Command create(ArmSubsystem arm, IntakeSubsystem intake) {
        //TODO - Tarun, shouldnt the IntakeBottomCommand set arm position to kMINArmAngle instead of kMAXArmAngle?
        return new RunCommand(
                () -> arm.setArmPosition(Constants.ArmConstants.kMaxArmAngle),
                // Dependencies:
                arm, intake)
            .until(arm::isArmDeployed)
            .withTimeout(Constants.AutoConstants.k_intakeBottomDuration)
            .finallyDo(arm::stop);
    }
}
