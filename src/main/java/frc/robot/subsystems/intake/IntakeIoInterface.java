package frc.robot.subsystems.intake;

/**
 * IO interface for the intake mechanism.
 *
 * <p>Abstracts hardware access so the subsystem works identically
 * with real hardware and with a physics simulation.
 */
public interface IntakeIoInterface {

    /** Mutable container for intake sensor readings. */
    class DeviceOutputs {
        /** Motor velocity in RPM. */
        public double velocityRPM;
        /** Motor output current in amps. */
        public double currentAmps;
    }

    /** Set the motor speed as a percentage (-1.0 to 1.0). */
    void setSpeed(double speed);

    /** Stop the motor immediately. */
    void stop();

    /** Read latest sensor data into the given outputs container. */
    void updateOutputs(DeviceOutputs outputs);
}
