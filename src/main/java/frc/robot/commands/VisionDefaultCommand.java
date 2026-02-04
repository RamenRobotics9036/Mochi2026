package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorDefaultCommandConstants;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Command for vision testing in sim
 * It is designed to be set as the default command for the {@link VisionSubsystem}.
 */
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class VisionDefaultCommand extends Command {
    private VisionSubsystem m_vision;

    /**
     * Creates a new VisonDefaultCommand.
     *
     * @param vision The vision subsystem to be supplied
     */
    public VisionDefaultCommand(VisionSubsystem vision) {
        m_vision = vision;

        addRequirements(m_vision);
    }

    @Override
    public void initialize() {
        // No initialization required; the subsystem maintains its current state
    }

    @Override
    public void execute() {
        boolean TagVisible = m_vision.isTagVisible();
        System.out.println(TagVisible);
    }

    /**
     * This command never ends naturally as it is a default command 
     * meant to provide constant manual control.
     */
    @Override
    public boolean isFinished() {
        // Default command runs indefinitely
        return false;
    }

    // /**
    //  * Stops elevator movement when the command is interrupted or cleared.
    //  * 
    //  * @param interrupted whether the command was interrupted/canceled
    //  */
    // @Override
    // public void end(boolean interrupted) {
    //     // Stop the elevator motor to prevent unintended movement
    //     //m_elevator.stopSystem();
    // }
}
