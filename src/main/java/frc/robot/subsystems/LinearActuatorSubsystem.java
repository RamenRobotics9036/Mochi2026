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

    /** The last position commanded to the servo (for dashboard comparison vs actual). */
    private double m_targetPosition = Double.NaN;

    /** Human-readable label of the last command sent. */
    private String m_lastCommand = "none";

    /** Constructs a new LinearActuatorSubsystem. */
    public LinearActuatorSubsystem() {
        m_actuator = new Servo(LinearActuatorConstants.kPwmChannel);

        // Publish static config values once so they're always visible in the dashboard
        SmartDashboard.putNumber("LinearActuator/Config/PwmChannel", LinearActuatorConstants.kPwmChannel);
        SmartDashboard.putNumber("LinearActuator/Config/ExtendedPosition", LinearActuatorConstants.kExtendedPosition);
        SmartDashboard.putNumber("LinearActuator/Config/RetractedPosition", LinearActuatorConstants.kRetractedPosition);
    }

    /** Extends the actuator to the fully extended position. */
    public void extend() {
        m_targetPosition = LinearActuatorConstants.kExtendedPosition;
        m_lastCommand = "extend";
        m_actuator.set(m_targetPosition);
        System.out.println("[LinearActuator] extend() called -> set(" + m_targetPosition + ") on PWM " + LinearActuatorConstants.kPwmChannel);
    }

    /** Retracts the actuator to the fully retracted position. */
    public void retract() {
        m_targetPosition = LinearActuatorConstants.kRetractedPosition;
        m_lastCommand = "retract";
        m_actuator.set(m_targetPosition);
        System.out.println("[LinearActuator] retract() called -> set(" + m_targetPosition + ") on PWM " + LinearActuatorConstants.kPwmChannel);
    }

    @Override
    public void periodic() {
        // Actual position read back from the servo object
        double actualPosition = m_actuator.get();
        // Raw servo angle (0–180 degrees) as the WPILib Servo class reports it
        double angleRaw = m_actuator.getAngle();

        SmartDashboard.putNumber("LinearActuator/ActualPosition", actualPosition);
        SmartDashboard.putNumber("LinearActuator/TargetPosition", Double.isNaN(m_targetPosition) ? -1.0 : m_targetPosition);
        SmartDashboard.putNumber("LinearActuator/AngleDeg", angleRaw);
        SmartDashboard.putString("LinearActuator/LastCommand", m_lastCommand);

        // Warn if actual position doesn't match target (servo may not be responding)
        if (!Double.isNaN(m_targetPosition)) {
            boolean mismatch = Math.abs(actualPosition - m_targetPosition) > 0.05;
            SmartDashboard.putBoolean("LinearActuator/PositionMismatch", mismatch);
        } else {
            SmartDashboard.putBoolean("LinearActuator/PositionMismatch", false);
        }
    }
}
