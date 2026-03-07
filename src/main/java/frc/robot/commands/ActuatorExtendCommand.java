package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.PidLinearActuator;

/**
 * One-shot command that fully extends an L16-R actuator.
 *
 * <p>Matches the proven CD pattern: call once in initialize(), the actuator's
 * internal controller handles the rest. No periodic update needed.
 *
 * <p>Usage in a subsystem or RobotContainer:
 * <pre>
 *   // Extend
 *   button.onTrue(new ActuatorExtendCommand(myActuator, true));
 *   // Retract
 *   button.onFalse(new ActuatorExtendCommand(myActuator, false));
 * </pre>
 */
public class ActuatorExtendCommand extends InstantCommand {

    /**
     * @param actuator the PidLinearActuator instance to command
     * @param extend   true = fully extend, false = fully retract
     */
    public ActuatorExtendCommand(PidLinearActuator actuator, boolean extend) {
        super(extend ? actuator::extendFully : actuator::retractFully);
        // If your actuator is owned by a subsystem, add the requirement:
        // addRequirements(subsystem);
    }
}
