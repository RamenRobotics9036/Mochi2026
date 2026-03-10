package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.IntakeConstants;
import frc.robot.sim.rollerssim.RollerIoInterface;

/**
 * Real-hardware implementation of {@link RollerIoInterface}.
 */
public class IntakeIoReal implements RollerIoInterface {
    private final SparkFlex m_intakeMotor;

    /** Constructor. */
    public IntakeIoReal() {
        SparkFlexConfig intakeConfig;

        m_intakeMotor = new SparkFlex(
            IntakeConstants.kIntakeMotorID,
            MotorType.kBrushless);

        intakeConfig = new SparkFlexConfig();
        intakeConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kIntakeRollerStallLimit)
            .inverted(true);
        m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    @Override
    public void stop() {
        m_intakeMotor.stopMotor();
    }

    @Override
    public void updateOutputs(DeviceOutputs outputs) {
        outputs.m_velocityRpm = m_intakeMotor.getEncoder().getVelocity();
        outputs.m_currentAmps = m_intakeMotor.getOutputCurrent();
    }
}
