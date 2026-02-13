package frc.robot.sim;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

/**
 * Processes raw joystick axes through the DriveSmooth pipeline and applies
 * simulation orientation correction when running in sim.
 *
 * <p>Encapsulates getDriveX / getDriveY / getDriveRotate plus the
 * sim-specific joystick transform so RobotContainer stays clean.
 */
public class JoystickXYZ {
    private final CommandXboxController m_controller;
    private final DriveSmooth m_driveSmooth = new DriveSmooth();
    private final double m_teleoperatedSpeed;
    private final double m_maxAngularRate;
    private final DoubleSupplier m_operatorForwardDegrees;
    private final SimJoystickOrientation m_simJoystickOrientation;

    /**
     * @param controller            driver Xbox controller
     * @param teleoperatedSpeed     max linear speed in m/s
     * @param maxAngularRate        max angular rate in rad/s
     * @param operatorForwardDegrees supplier for the operator-relative forward
     *                               direction (degrees) — used only in simulation
     */
    public JoystickXYZ(
            CommandXboxController controller,
            double teleoperatedSpeed,
            double maxAngularRate,
            DoubleSupplier operatorForwardDegrees) {

        m_controller = controller;
        m_teleoperatedSpeed = teleoperatedSpeed;
        m_maxAngularRate = maxAngularRate;
        m_operatorForwardDegrees = operatorForwardDegrees;
        m_simJoystickOrientation = Robot.isSimulation()
            ? new SimJoystickOrientation()
            : null;
    }

    /**
     * Returns fully-processed joystick inputs (deadband, smoothing, sim correction).
     *
     * @return a {@link JoystickInputsRecord} with drive X (m/s), drive Y (m/s),
     *         and rotation (rad/s)
     */
    public JoystickInputsRecord getJoystickInputs() {
        double leftX = getDriveX();
        double leftY = getDriveY();
        double rightX = getDriveRotate();

        // $VISIONSIM — correct joystick orientation for sim
        if (Robot.isSimulation()) {
            return m_simJoystickOrientation.transformJoystickOrientation(
                m_operatorForwardDegrees.getAsDouble(),
                leftX,
                leftY,
                rightX);
        }

        return new JoystickInputsRecord(leftX, leftY, rightX);
    }

    // ---- internal pipeline methods ----

    private double getDriveX() {
        double input = m_driveSmooth.processTranslationX(-m_controller.getLeftY());
        double inputScale = m_controller.rightBumper().getAsBoolean() ? 0.5 : 1.0;
        return input * m_teleoperatedSpeed * inputScale;
    }

    private double getDriveY() {
        double input = m_driveSmooth.processTranslationY(-m_controller.getLeftX());
        double inputScale = m_controller.rightBumper().getAsBoolean() ? 0.5 : 1.0;
        return input * m_teleoperatedSpeed * inputScale;
    }

    private double getDriveRotate() {
        double input = m_driveSmooth.processRotation(-m_controller.getRightX());
        double inputScale = m_controller.rightBumper().getAsBoolean() ? 0.5 : 1.0;
        return input * m_maxAngularRate * inputScale;
    }
}
