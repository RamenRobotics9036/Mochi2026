package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives an Actuonix L16-R (or similar) linear servo by writing
 * pulse-width microseconds directly — no WPILib Servo bounds/deadband
 * mapping that can silently clip the output.
 */
public class LinearServo {
    private final PWM m_pwm;
    private final double m_speed;
    private final double m_length;
    private final int m_minPulseUs;
    private final int m_maxPulseUs;
    private double setPos;
    private double curPos;
    private double lastTime;
    private final String m_name;

    private static final double POSITION_TOLERANCE_MM = 0.5;
    /** Default step size for incremental hood adjustments [mm] */
    public static final double DEFAULT_STEP_MM = 5.0;

    /**
     * @param channel    PWM channel used to control the servo
     * @param length     max length of the servo [mm]
     * @param speed      max speed of the servo [mm/second]
     * @param minPulseUs minimum PWM pulse width in microseconds (fully retracted)
     * @param maxPulseUs maximum PWM pulse width in microseconds (fully extended)
     */
    public LinearServo(int channel, int length, int speed, int minPulseUs, int maxPulseUs) {
        m_pwm = new PWM(channel);
        m_length = length;
        m_speed = speed;
        m_minPulseUs = minPulseUs;
        m_maxPulseUs = maxPulseUs;
        m_name = "LinearServo[" + channel + "]";

        // Set the PWM output period — standard hobby servos use 50 Hz (20 ms)
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);

        setPos = 0.0;
        curPos = 0.0;
        lastTime = Timer.getFPGATimestamp();

        // Command the initial position immediately
        writePulse();
    }

    /**
     * Convenience constructor using default L16-R pulse range (1000–2000 µs).
     */
    public LinearServo(int channel, int length, int speed) {
        this(channel, length, speed, 1000, 2000);
    }

    /** Extend the actuator by one default step increment. */
    public void stepExtend() {
        setLinearPosition(setPos + DEFAULT_STEP_MM);
    }

    /** Extend the actuator by a custom step size. */
    public void stepExtend(double stepMm) {
        setLinearPosition(setPos + Math.abs(stepMm));
    }

    /** Retract the actuator by one default step increment. */
    public void stepRetract() {
        setLinearPosition(setPos - DEFAULT_STEP_MM);
    }

    /** Retract the actuator by a custom step size. */
    public void stepRetract(double stepMm) {
        setLinearPosition(setPos - Math.abs(stepMm));
    }

    /**
     * Command the actuator to a position in mm.
     *
     * @param setpoint the target position [mm]
     */
    public void setLinearPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        writePulse();
    }

    /** Maps setPos to a pulse width and writes it directly to the PWM output. */
    private void writePulse() {
        double fraction = (m_length > 0) ? setPos / m_length : 0.0;
        int pulseUs = (int) Math.round(m_minPulseUs + fraction * (m_maxPulseUs - m_minPulseUs));
        m_pwm.setPulseTimeMicroseconds(pulseUs);
    }

    /** Update the estimated current position based on speed and elapsed time. */
    public void updateCurPos() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastTime;

        if (curPos > setPos + m_speed * dt) {
            curPos -= m_speed * dt;
        } else if (curPos < setPos - m_speed * dt) {
            curPos += m_speed * dt;
        } else {
            curPos = setPos;
        }

        lastTime = currentTime;
    }

    /** Publish diagnostic telemetry to SmartDashboard. */
    public void updateTelemetry() {
        double fraction = (m_length > 0) ? setPos / m_length : 0.0;
        int pulseUs = (int) Math.round(m_minPulseUs + fraction * (m_maxPulseUs - m_minPulseUs));

        SmartDashboard.putNumber(m_name + "/SetpointMM", setPos);
        SmartDashboard.putNumber(m_name + "/EstimatedPosMM", curPos);
        SmartDashboard.putNumber(m_name + "/FractionCmd", fraction);
        SmartDashboard.putNumber(m_name + "/PWM_us", pulseUs);
        SmartDashboard.putBoolean(m_name + "/IsFinished", isFinished());
    }

    /** Current estimated position [mm]. Must call {@link #updateCurPos()} periodically. */
    public double getLinearPosition() {
        return curPos;
    }

    /** @return Servo target position [mm] */
    public double getSetpoint() {
        return setPos;
    }

    /** @return The fraction [0.0–1.0] currently being commanded. */
    public double getCommandedPercent() {
        return (m_length > 0) ? setPos / m_length : 0.0;
    }

    /** @return true when estimated position is within tolerance of the target. */
    public boolean isFinished() {
        return Math.abs(curPos - setPos) < POSITION_TOLERANCE_MM;
    }
}