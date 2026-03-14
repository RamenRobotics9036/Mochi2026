package frc.robot.sim.elevatorssim;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Simulated implementation of {@link ElevatorIoInterface} using
 * {@link ElevatorSim} for physically accurate gravity modeling.
 *
 * <p>Position is reported in meters and naturally clamped to the
 * min/max heights supplied at construction.
 */
public class ElevatorIoSim implements ElevatorIoInterface {
    private static final DCMotor kMotor = DCMotor.getNeoVortex(1);
    private static final double kLoopSeconds = 0.02;

    private final ElevatorSim m_elevatorSim;

    // Sim device entries visible in the WPILib sim GUI
    private final SimDevice m_simDevice;
    private final SimDouble m_simPosition;
    private final SimDouble m_simCurrent;

    /**
     * Constructs the simulated elevator IO.
     *
     * @param deviceName      name shown in the WPILib sim GUI
     * @param gearRatio       motor-to-mechanism gear ratio
     * @param carriageMassKg  mass of the carriage (and anything it lifts) in kg
     * @param drumRadiusMeters radius of the winch drum in meters
     * @param minHeightMeters  minimum carriage height in meters
     * @param maxHeightMeters  maximum carriage height in meters
     */
    public ElevatorIoSim(
            String deviceName,
            double gearRatio,
            double carriageMassKg,
            double drumRadiusMeters,
            double minHeightMeters,
            double maxHeightMeters) {
        m_elevatorSim = new ElevatorSim(
            kMotor,
            gearRatio,
            carriageMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            true,
            minHeightMeters);

        m_simDevice = SimDevice.create(deviceName);
        m_simPosition = m_simDevice.createDouble(
            "Position Meters", Direction.kOutput, 0.0);
        m_simCurrent = m_simDevice.createDouble(
            "Current Amps", Direction.kOutput, 0.0);
    }

    @Override
    public void setSpeed(double speed) {
        m_elevatorSim.setInputVoltage(
            MathUtil.clamp(speed, -1.0, 1.0) * 12.0);
    }

    @Override
    public void stop() {
        m_elevatorSim.setInputVoltage(0.0);
    }

    @Override
    public void updateOutputs(DeviceOutputs outputs) {
        m_elevatorSim.update(kLoopSeconds);

        outputs.m_positionMeters = m_elevatorSim.getPositionMeters();
        outputs.m_currentAmps = m_elevatorSim.getCurrentDrawAmps();

        m_simPosition.set(outputs.m_positionMeters);
        m_simCurrent.set(outputs.m_currentAmps);
    }
}
