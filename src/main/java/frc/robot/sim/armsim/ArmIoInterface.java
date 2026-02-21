package frc.robot.sim.armsim;

/**
 * IO interface for the intake arm mechanism.
 */
public interface ArmIoInterface {
    /** Mutable container for arm telemetry. */
    class DeviceOutputs {
        /** Arm position in subsystem units. */
        public double position;
        /** Arm velocity in subsystem units per second. */
        public double velocity;
        /** Motor output current in amps. */
        public double currentAmps;
    }

    /** Immediately start moving arm [-1.0, 1.0]. */
    void moveArmWithSpeed(double speed);

    /** Command closed-loop position in subsystem units. */
    void setPosition(double position);

    
    /**Gets the position, multiplied by gear ratio */
    double getPosition();

    /**Gets the raw encoder value */
    double getEncoderPosition();

    /** Reset the arm encoder position to 0. */
    void resetEncoderValue();

    /** Stop arm motion immediately. */
    void stop();

    /** Read latest arm telemetry. */
    void updateOutputs(DeviceOutputs outputs);
}
