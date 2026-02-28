package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Time;

public class HoodActuator extends Servo {
    /** The maximum speed of the linear actuator in mm/s */
    double m_speed;
    /** The maximum length of the linear actuator in mm */
    double m_length;

    /** The target position of the linear actuator in mm */
    double setPos;
    /** The hypothetical current position of the linear actuator in mm */
    double curPos;

    /** The FPGA timestamp as of when updateCurPos was last called */
    double lastTime = 0;

    /**
     * Parameters for L-16 Actuonix Linear Actuators
     * 
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/s]
     */
    public HoodActuator(int channel, int length, int speed) {
        super(channel);
        setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        m_length = length;
        m_speed = speed;
        lastTime = Timer.getFPGATimestamp(); // Just in case the HoodActuator isn't created exactly at 0 seconds
    }

    /**
     * Set the target position of the servo
     * 
     * @param setpoint the target position of the servo [mm]
     */
    public void setPosition(double setpoint) {
        updateCurPos(); // The example code didn't have this, which seems like an oversight since setpoint is used to calculate curPos
        setPos = MathUtil.clamp(setpoint, 0, m_length); // Makes sure the setpoint is within the max length of the servo
        set(setPos/m_length); // Scales it to work with set()
    }

    /**
     * Run this method in any periodic function to update the position estimation of the servo
     */
    public void updateCurPos() {
        /** Time since last called */
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed * dt) {
            // If the servo would have moved in the negative direction towards
            // the setpoint and not reached it, adjust the current position to match
            curPos -= m_speed * dt;
        } else if (curPos < setPos - m_speed * dt) {
            // If the servo would have moved in the positive direction towards
            // the setpoint and not reached it, adjust the current position to match
            curPos += m_speed * dt;
        } else {
            // Otherwise we can conclude that the servo has reached the setpoint
            curPos = setPos;
        }

        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Current position of the servo
     * The example code suggests that this only works if {@link #updateCurPos() updateCurPos()} is called periodically
     * However, I've made sure to call that in key spots within the subsystem itself, so that *hypothetically* shouldn't be true
     * I would still call it periodically just to be safe though
     * 
     * @return Servo Position [mm]
     */
    public double getPosition() {
        updateCurPos(); // The original code didn't have this, which seems like an oversight, since this makes sure the returned value is up-to-date
        return curPos;
    }

    /**
     * Checks if the servo is at its target position
     * The example code suggests that this only works if {@link #updateCurPos() updateCurPos()} is called periodically
     * However, I've made sure to call that in key spots within the subsystem itself, so that *hypothetically* shouldn't be true
     * I would still call it periodically just to be safe though
     * 
     * @return true when servo is at its target
     */
    public boolean isFinished() {
        updateCurPos(); // The original code didn't have this, which seems like an oversight, since this makes sure the returned value is up-to-date
        return curPos == setPos;
    }
}