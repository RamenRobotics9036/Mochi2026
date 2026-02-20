package frc.robot.sim.armsim;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;

/**
 * Simulated implementation of {@link ArmIoInterface} for a simple up/down arm.
 */
public class ArmIoSim implements ArmIoInterface {
    private static final DCMotor kArmMotor = DCMotor.getNeoVortex(2);
    private static final double kNominalLoopSeconds = 0.02;
    private static final double kPositionKpVoltsPerUnit = 0.25;

    private final SingleJointedArmSim m_armSim;
    private final SimDevice m_simDevice;
    private final SimDouble m_simPosition;
    private final SimDouble m_simVelocity;
    private final SimDouble m_simCurrent;

    private boolean m_positionControlEnabled = false;
    private double m_targetPosition = 0.0;
    private double m_openLoopInputVolts = 0.0;

    /** Constructor. */
    public ArmIoSim(
            String deviceName,
            double momentOfInertia,
            double armLengthMeters,
            double gearRatio) {
        m_armSim = new SingleJointedArmSim(
            kArmMotor,
            gearRatio,
            momentOfInertia,
            armLengthMeters,
            Units.degreesToRadians(IntakeConstants.kMinArmAngle),
            Units.degreesToRadians(IntakeConstants.kMaxArmAngle),
            true,
            Units.degreesToRadians(IntakeConstants.kMinArmAngle));

        m_simDevice = SimDevice.create(deviceName);
        if (m_simDevice != null) {
            m_simPosition = m_simDevice.createDouble("Position", Direction.kOutput, 0.0);
            m_simVelocity = m_simDevice.createDouble("Velocity", Direction.kOutput, 0.0);
            m_simCurrent = m_simDevice.createDouble("Current Amps", Direction.kOutput, 0.0);
        } else {
            m_simPosition = null;
            m_simVelocity = null;
            m_simCurrent = null;
        }
    }

    @Override
    public void moveArmWithSpeed(double speed) {
        m_positionControlEnabled = false;
        m_openLoopInputVolts = MathUtil.clamp(speed, -1.0, 1.0) * 12.0;
    }

    @Override
    public void setPosition(double position) {
        m_positionControlEnabled = true;
        m_targetPosition = position;
    }

    @Override
    public void resetEncoderValue() {
        m_armSim.setState(Units.degreesToRadians(0.0), 0.0);
    }

    @Override
    public void stop() {
        m_positionControlEnabled = false;
        m_openLoopInputVolts = 0.0;
    }

    @Override
    public void updateOutputs(DeviceOutputs outputs) {
        if (m_positionControlEnabled) {
            double positionError = m_targetPosition - Units.radiansToDegrees(m_armSim.getAngleRads());
            m_openLoopInputVolts = MathUtil.clamp(
                positionError * kPositionKpVoltsPerUnit,
                -12.0,
                12.0);
        }

        m_armSim.setInputVoltage(m_openLoopInputVolts);
        m_armSim.update(kNominalLoopSeconds);

        outputs.position = Units.radiansToDegrees(m_armSim.getAngleRads());
        outputs.velocity = Units.radiansToDegrees(m_armSim.getVelocityRadPerSec());
        outputs.currentAmps = m_armSim.getCurrentDrawAmps();

        if (m_simPosition != null) {
            m_simPosition.set(outputs.position);
            m_simVelocity.set(outputs.velocity);
            m_simCurrent.set(outputs.currentAmps);
        }
    }
}
