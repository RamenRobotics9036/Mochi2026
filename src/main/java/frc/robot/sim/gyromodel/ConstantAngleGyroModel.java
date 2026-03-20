package frc.robot.sim.gyromodel;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Minimal integration-test gyro model.
 *
 * <p>This intentionally reports a fixed heading so we can verify the robot stack
 * is consuming our simulated gyro path before adding a physical source.
 */
public class ConstantAngleGyroModel implements SimGyroModel {
    private static final Rotation2d kConstantHeading = Rotation2d.fromDegrees(45);

    @Override
    public void reset() {}

    @Override
    public void update(double dtSeconds) {}

    @Override
    public Rotation2d getMeasuredHeading() {
        return kConstantHeading;
    }

    @Override
    public double getMeasuredOmegaRadPerSec() {
        return 0.0;
    }
}
