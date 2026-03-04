package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.commands.IntakeArmHomeCommand;
import frc.robot.commands.SetIntakeBottomCommand;

public class TestSubsystems {
    /** Temporary sequence command for wiring/testing. */
    public static Command test(
            IntakeSubsystem intakeSubsystem,
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            ClimberSubsystem climberSubsystem) {
        // For now, we ONLY allow running this in simulation, since it hasn't been tested
        // on the real robot yet.
        if (Robot.isSimulation()) {
            return Commands.sequence(
                Commands.print("Hello"),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> intakeSubsystem.setIntakeSpeed(IntakeConstants.kIntakeSpeed),
                    intakeSubsystem),
                Commands.waitSeconds(2.0),
                Commands.runOnce(intakeSubsystem::stop, intakeSubsystem),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> indexerSubsystem.setSpeed(IndexerConstants.kIndexSpeed),
                    indexerSubsystem),
                Commands.waitSeconds(2.0),
                Commands.runOnce(indexerSubsystem::stop, indexerSubsystem),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> shooterSubsystem.setSpeed(ShooterConstants.kShootSpeed),
                    shooterSubsystem),
                Commands.waitSeconds(2.0),
                Commands.runOnce(shooterSubsystem::stop, shooterSubsystem),
                Commands.waitSeconds(1.0),
                Commands.runOnce(armSubsystem::beginHoming, armSubsystem),
                Commands.waitUntil(armSubsystem::isArmHomed),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> armSubsystem.setArmPosition(ArmConstants.kMaxArmAngle),
                    armSubsystem),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> climberSubsystem.setClimbSpeed(ClimberConstants.kClimbUpSpeed),
                    climberSubsystem),
                Commands.waitSeconds(0.5),
                Commands.runOnce(
                    () -> climberSubsystem.setClimbSpeed(ClimberConstants.kClimbDownSpeed),
                    climberSubsystem),
                Commands.waitSeconds(0.5),
                Commands.runOnce(climberSubsystem::stop, climberSubsystem),
                Commands.print("End")
            );
        }
        else {
            return Commands.none();
        }
    }

    public static Command test_homing(
            IntakeSubsystem intakeSubsystem,
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            ClimberSubsystem climberSubsystem) {
        // For now, we ONLY allow running this in simulation, since it hasn't been tested
        // on the real robot yet.
        if (Robot.isSimulation()) {
            return Commands.sequence(
                Commands.print("Starting homing..."),
                IntakeArmHomeCommand.create(armSubsystem),
                Commands.waitUntil(armSubsystem::isArmHomed),
                Commands.print("Homing complete. Raising arm to top..."),
                SetIntakeBottomCommand.create(armSubsystem, intakeSubsystem),
                Commands.print("Arm at top. Starting second homing..."),
                IntakeArmHomeCommand.create(armSubsystem),
                Commands.waitUntil(armSubsystem::isArmHomed),
                Commands.print("Second homing complete.")
            );
        }
        else {
            return Commands.none();
        }
    }
}
