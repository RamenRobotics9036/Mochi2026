package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LinearServo extends Servo {
    private final double m_speed;
    private final double m_length;
    private final int m_minPulseUs;
    private final int m_maxPulseUs;
    private double setPos;
    private double curPos;
    private double lastTime;
    private final String m_name;

    /** Position tolerance, used solely for the isFinished() function [mm] */
    private static final double POSITION_TOLERANCE_MM = 0.5;
    /** Default step size for incremental hood adjustments [mm] */
    public static final double DEFAULT_STEP_MM = 5.0;

    /** Creates a new linear actuator servo
     * 
     * @param channel PWM channel for the linear actuator servo
     * @param length Max length for the linear actuator servo [mm]
     * @param speed Max no-load speed for the linear actuator servo [mm/s]
     */
    public LinearServo(int channel, int length, int speed, int minPulseUs, int maxPulseUs) {
        super(channel);
        m_length = length;
        m_speed = speed;
        m_minPulseUs = minPulseUs;
        m_maxPulseUs = maxPulseUs;
        m_name = "LinearServo[" + channel + "]";

        setBoundsMicroseconds(maxPulseUs, maxPulseUs, (maxPulseUs + minPulseUs) / 2, minPulseUs, minPulseUs);

        setPos = 0.0;
        curPos = 0.0;
        lastTime = Timer.getFPGATimestamp();
    }

    public LinearServo(int channel, int length, int speed) {
        this(channel, length, speed, 1000, 2000);
    }

    /**
     * Extend the actuator by one step increment.
     * Call this on a button press (not held) for discrete control.
     */
    public void stepExtend() {
        System.out.println("Setting length to " + Double.toString(setPos - DEFAULT_STEP_MM) + " mm");
        setLinearPosition(setPos + DEFAULT_STEP_MM);
    }

    /**
     * Extend the actuator by a custom step size.
     * @param stepMm step size in mm (positive = extend)
     */
    public void stepExtend(double stepMm) {
        System.out.println("Setting length to " + Double.toString(setPos + Math.abs(stepMm)) + " mm using custom increments");
        setLinearPosition(setPos + Math.abs(stepMm));
    }

    /**
     * Retract the actuator by one step increment.
     * Call this on a button press (not held) for discrete control.
     */
    public void stepRetract() {
        setLinearPosition(setPos - DEFAULT_STEP_MM);
    }

    /**
     * Retract the actuator by a custom step size.
     * @param stepMm step size in mm (positive value = retract distance)
     */
    public void stepRetract(double stepMm) {
        setLinearPosition(setPos - Math.abs(stepMm));
    }

    public void setLinearPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        double fraction = setPos / m_length;


        super.setPosition(fraction);
        System.out.println("Raw mm setpoint: " + Double.toString(setpoint) + ", clamped setpoint: " + Double.toString(setPos) + ", final fraction: " + Double.toString(fraction));
        System.out.println("---");
    }

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

    public void updateTelemetry() {
        SmartDashboard.putNumber(m_name + "/SetpointMM", setPos);
        SmartDashboard.putNumber(m_name + "/EstimatedPosMM", curPos);
        SmartDashboard.putNumber(m_name + "/FractionCmd", setPos / m_length);
        SmartDashboard.putNumber(m_name + "/PWM_us",
                m_minPulseUs + (setPos / m_length) * (m_maxPulseUs - m_minPulseUs));
        SmartDashboard.putBoolean(m_name + "/IsFinished", isFinished());
    }

    public double getLinearPosition() {
        return curPos;
    }

    public double getSetpoint() {
        return setPos;
    }

    public double getCommandedPercent() {
        return get();
    }

    public boolean isFinished() {
        return Math.abs(curPos - setPos) < POSITION_TOLERANCE_MM;
    }
}