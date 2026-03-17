package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

/** Full autonomous climb sequence: extend, retract, then stop. */
public class FullAutoClimbCommand {
    /** Returns a Command that runs the full auto climb sequence. */
    public static Command create(ClimberSubsystem climber) {
        return Commands.sequence(
                new RunCommand(
                    () -> climber.setClimbSpeed(Constants.ClimberConstants.kClimbUpSpeed),
                    climber).withTimeout(Constants.AutoConstants.k_raiseClimbDuration),
                new RunCommand(
                    () -> climber.setClimbSpeed(Constants.ClimberConstants.kClimbDownSpeed),
                    climber).withTimeout(Constants.AutoConstants.k_lowerClimbDuration))
            .finallyDo(climber::stop);
    }
}
