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

    private static final double POSITION_TOLERANCE_MM = 0.5;

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

    public void setLinearPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        double fraction = setPos / m_length;
        super.setPosition(fraction);
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

    public double getPosition() {
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