package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LinearActuatorConstants;

/**
 * Subsystem for the WCP-0408 100 mm stroke linear servo actuator.
 *
 * <p>The actuator is controlled via a standard PWM servo signal.
 * Position 0.0 maps to fully retracted; 1.0 maps to fully extended.
 */
public class LinearActuatorSubsystem extends SubsystemBase {
    private final Servo m_actuator;

    /** Constructs a new LinearActuatorSubsystem. */
    public LinearActuatorSubsystem() {
        m_actuator = new Servo(LinearActuatorConstants.kPwmChannel);
    }

    /** Extends the actuator to the fully extended position. */
    public void extend() {
        m_actuator.set(LinearActuatorConstants.kExtendedPosition);
    }

    /** Retracts the actuator to the fully retracted position. */
    public void retract() {
        m_actuator.set(LinearActuatorConstants.kRetractedPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LinearActuator/Position", m_actuator.get());
    }
}
