package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class LinearServo extends Servo {
    private double m_speed;
    private double m_length;
    private double setPos;
    private double curPos;
    private double lastTime;

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length  max length of the servo [mm]
     * @param speed   max speed of the servo [mm/second]
     */
    public LinearServo(int channel, int length, int speed) {
        super(channel);
        setBoundsMicroseconds(1000, 1200, 1500, 1800, 2000);
        m_length = length;
        m_speed = speed;
        // Need to initialize timing properly, else on very first update 'dt' will be since robot boot.
        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Run this method in any periodic function to update the position estimation of your servo
     *
     * @param setpoint the target position of the servo [mm]
     */
    public void setPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed((setPos / m_length * 2) - 1);
    }

    /**
     * Run this method in any periodic function to update the position estimation of your servo
     */
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

    /**
     * Current position of the servo, must be calling {@link #updateCurPos() updateCurPos()} periodically
     *
     * @return Servo Position [mm]
     */
    public double getPosition() {
        return curPos;
    }

    /**
     * Gets the currently commanded target position.
     *
     * @return Servo target position [mm]
     */
    public double getSetpoint() {
        return setPos;
    }

    /**
     * Checks if the servo is at its target position, must be calling {@link #updateCurPos() updateCurPos()} periodically
     *
     * @return true when servo is at its target
     */
    public boolean isFinished() {
        return curPos == setPos;
    }
}
