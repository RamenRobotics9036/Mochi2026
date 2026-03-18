package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** Shoot. */
public class ShootCommand {
    /** Returns a Command that shoots. */
    public static Command create(
        ShooterSubsystem shooter,
        IndexerSubsystem indexer) {

        return Commands.sequence(
            // Spin up shooter
            new RunCommand(() -> {
                shooter.setSpeed(Constants.ShooterConstants.kShootSpeed);
            },
            // Dependencies:
            shooter, indexer)
            .withTimeout(Constants.IndexerConstants.kIndexDelay),

            // Start the indexer and start shooting
            new RunCommand(() -> {
                shooter.setSpeed(Constants.ShooterConstants.kShootSpeed);
                indexer.setSpeed(Constants.IndexerConstants.kIndexSpeed);
            },
            // Dependencies:
            shooter, indexer)
            .withTimeout(Constants.AutoConstants.k_shootCommandTotalDuration - Constants.IndexerConstants.kIndexDelay)
        )
        
        // finallyDo ensures motors are stopped even if command is cancelled.
        .finallyDo(() -> {
            shooter.stop();
            indexer.stop();
        });
    }
}
