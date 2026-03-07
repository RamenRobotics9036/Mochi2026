package frc.robot.util;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class LinearActuatorServo extends Servo {

    /** The desired position for the servo, scaled from 0 to 1 */
    private double m_desiredPosition;
    /** The estimated current position for the servo, scaled from 0 to 1 */
    private double m_currentPosition;

    /** 
     * Creates a new linear actuator servo
     * 
     * @param channel The PWM channel for the servo
     */
    public LinearActuatorServo(int channel) {
        super(channel);

        // The actuator accepts values from 1000 to 2000 microseconds, so we set the bounds to be that
        // There should be no deadband, because it's unnecessay for this type of servo
        setBoundsMicroseconds(1000, 1500, 1500, 1500, 2000);
    }

    /** 
     * Sets the target position of the servo
     * 
     * @param setpoint The desired position for the servo, scaled from 0 to 1
     */
    @Override
    public void set(double setpoint) {
        m_desiredPosition = setpoint;
    }

    /**
     * The periodic function for the servo
     * Needs to be called in the periodic function of the subsystem it belongs to since servos can't have periodic functions
     */
    public void periodic() {
        //TODO: Finish this function
    }
}
