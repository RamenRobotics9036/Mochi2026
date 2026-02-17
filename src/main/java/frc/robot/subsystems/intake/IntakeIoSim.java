package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

/**
 * Simulated implementation of {@link IntakeIoInterface} using a {@link FlywheelSim}.
 *
 * <p>Physics are modeled as a single-NEO flywheel. A {@link SimDevice}
 * exposes velocity and current in the WPILib sim GUI for debugging.
 */
public class IntakeIoSim implements IntakeIoInterface {
    /** NEO motor model (single motor). */
    private static final DCMotor kMotor = DCMotor.getNEO(1);
    /** Gear ratio (motor rotations per mechanism rotation). */
    private static final double kGearing = 1.0;
    /** Moment of inertia of the intake roller in kg·m². */
    private static final double kMoiKgM2 = 0.001;

    private final FlywheelSim m_flyWheelSim;
    private double m_appliedSpeed = 0.0;

    // Sim device entries visible in the WPILib sim GUI
    private final SimDevice m_simDevice;
    private final SimDouble m_simVelocity;
    private final SimDouble m_simCurrent;
    private final SimDouble m_simAppliedOutput;

    /** Constructs the simulated intake IO. */
    public IntakeIoSim() {
        m_flyWheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(kMotor, kMoiKgM2, kGearing),
            kMotor
        );

        // Register a SimDevice so values appear in the sim GUI
        m_simDevice = SimDevice.create("IntakeSim");
        m_simVelocity = m_simDevice.createDouble(
            "Velocity RPM", Direction.kOutput, 0.0);
        m_simCurrent = m_simDevice.createDouble(
            "Current Amps", Direction.kOutput, 0.0);
        m_simAppliedOutput = m_simDevice.createDouble(
            "Applied Output", Direction.kOutput, 0.0);
    }

    @Override
    public void setSpeed(double speed) {
        m_appliedSpeed = speed;
        m_flyWheelSim.setInputVoltage(speed * 12.0);
    }

    @Override
    public void stop() {
        m_appliedSpeed = 0.0;
        m_flyWheelSim.setInputVoltage(0.0);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // Step the physics simulation forward by one robot loop period
        m_flyWheelSim.update(0.02);

        inputs.velocityRPM = m_flyWheelSim.getAngularVelocityRPM();
        inputs.currentAmps = m_flyWheelSim.getCurrentDrawAmps();
        inputs.appliedOutput = m_appliedSpeed;

        // Push values into the sim GUI
        m_simVelocity.set(inputs.velocityRPM);
        m_simCurrent.set(inputs.currentAmps);
        m_simAppliedOutput.set(inputs.appliedOutput);
    }
}
