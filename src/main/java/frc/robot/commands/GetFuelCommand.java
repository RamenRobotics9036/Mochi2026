package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                if (arm.getArmAngle() > Constants.ArmConstants.kMaxArmAngle) {
                    DriverStation.reportWarning("ArmSubsystem: Current arm angle greater than the maximum! Check the encoder.", false);
                    SmartDashboard.putString("Intake/ArmWarning", "Current arm angle greater than the maximum! Check the encoder.");
                }
                if (arm.getArmAngle() < Constants.ArmConstants.kMinArmAngle) {
                    DriverStation.reportWarning("ArmSubsystem: Current arm angle less than the minimum! Check the encoder.", false);
                    SmartDashboard.putString("Intake/ArmWarning", "Current arm angle less than the minimum! Check the encoder.");
                }

                //TODO: automatically incorporate gear ratio
                arm.setArmPosition(Constants.ArmConstants.kMaxArmAngle);
                intake.setIntakeSpeed(Constants.IntakeConstants.kIntakeSpeed);
                DriverStation.reportWarning("Command being run!", false);
                DriverStation.reportWarning("Target speed: "+Double.toString(Constants.ArmConstants.kMaxArmAngle), false);
            },
            // Dependencies:
            arm, intake)
            .withTimeout(Constants.AutoConstants.k_getFuelDuration)
            .finallyDo(() -> {
                arm.stop();
                intake.stop();
                DriverStation.reportWarning("Command stopped!!", false);
            });
    }
}
