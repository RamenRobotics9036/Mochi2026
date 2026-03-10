package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.IndexerConstants;
import frc.robot.sim.rollerssim.RollerIoInterface;

/**
 * Real-hardware implementation of {@link RollerIoInterface} for the indexer.
 */
public class IndexerIoReal implements RollerIoInterface {
    private final SparkFlex m_motor;

    /** Constructor. */
    public IndexerIoReal() {
        m_motor = new SparkFlex(IndexerConstants.kMotorID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IndexerConstants.kIndexerCurrentLimit)
            .inverted(true);

        m_motor.configure(config, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void updateOutputs(DeviceOutputs outputs) {
        outputs.m_velocityRpm = m_motor.getEncoder().getVelocity();
        outputs.m_currentAmps = m_motor.getOutputCurrent();
    }
}
