package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.botconfig.BotConfigInterface;

/**
 * Real-hardware implementation of {@link IntakeIoInterface} backed by a REV SparkMax.
 */
public class IntakeIoReal implements IntakeIoInterface {
    private final SparkMax m_motor;

    /** Constructs the real intake IO with the given bot configuration. */
    public IntakeIoReal(BotConfigInterface config) {
        m_motor = new SparkMax(
            config.getIntakeMotorId(), MotorType.kBrushless);

        var cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(config.getIntakeStallLimit());

        m_motor.configure(cfg,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
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
    public void updateInputs(IntakeInputs inputs) {
        inputs.velocityRPM = m_motor.getEncoder().getVelocity();
        inputs.currentAmps = m_motor.getOutputCurrent();
        inputs.appliedOutput = m_motor.getAppliedOutput();
    }
}
