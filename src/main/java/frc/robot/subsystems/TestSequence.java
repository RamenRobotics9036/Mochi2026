package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;

public class TestSequence {
    /** Temporary sequence command for wiring/testing. */
    public static Command testSubsystems(
            IntakeSubsystem intakeSubsystem,
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem) {
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
                Commands.print("End")
            );
        }
        else {
            return Commands.none();
        }
    }
}
