package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * IntakeCommand runs the intake motor until a stall is detected (indicating a successful intake)
 * or until the command is interrupted.
 * 
 * <p>When a piece is grabbed (stall detected), the controller rumbles to provide
 * tactile feedback to the driver.
 */
public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final CommandXboxController m_controller;
    private boolean m_grabbedPiece = false;

    /**
     * Creates an IntakeCommand with haptic feedback.
     * 
     * @param intake The intake subsystem
     * @param controller The driver's Xbox controller for rumble feedback
     */
    public IntakeCommand(IntakeSubsystem intake, CommandXboxController controller) {
        m_intake = intake;
        m_controller = controller;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_grabbedPiece = false;
    }

    @Override
    public void execute() {
        m_intake.setSpeed(IntakeConstants.kIntakeSpeed);
    }

    @Override
    public boolean isFinished() {
        // Stop automatically if we sense a stall (ball/piece acquired)
        if (m_intake.isStalled()) {
            m_grabbedPiece = true;
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        
        // Provide haptic feedback if we successfully grabbed a piece
        if (m_grabbedPiece && !interrupted) {
            rumbleController();
        }
    }

    /**
     * Rumbles the left side of the controller to indicate a successful grab.
     */
    private void rumbleController() {
        // Start rumble on left motor (strong, 500ms feel)
        m_controller.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
        
        // Schedule rumble stop after 500ms using a separate thread
        new Thread(() -> {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            m_controller.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
        }).start();
    }
}
