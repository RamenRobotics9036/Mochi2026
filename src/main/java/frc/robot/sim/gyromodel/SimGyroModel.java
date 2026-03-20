package frc.robot.sim.gyromodel;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Simulation model for a gyro sensor.
 *
 * <p>The gyro model is intentionally separated from drivetrain pose simulation so
 * simulated heading can diverge from the drivetrain's internal perfect sim state.
 */
public interface SimGyroModel {

    /** Resets any internal state in the gyro model. */
    void reset();

    /**
     * Advances the gyro model by one simulation step.
     *
     * @param dtSeconds Time delta for the current sim step
     */
    void update(double dtSeconds);

    /**
     * Returns the simulated measured heading that the gyro should report.
     *
     * @return Simulated gyro yaw
     */
    Rotation2d getMeasuredHeading();

    /**
     * Returns the simulated measured angular velocity.
     *
     * @return Simulated gyro yaw rate in radians per second
     */
    double getMeasuredOmegaRadPerSec();
}
