package frc.robot.subsystems.spinny;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.SpinnyWheelsConstants;
import robotutils.pub.interfaces.simio.RollerIoInterface;


/**
 * Real-hardware implementation of {@link RollerIoInterface} for the spinny wheels.
 */
public class SpinnyIoReal implements RollerIoInterface {
    private final SparkFlex m_motor = new SparkFlex(SpinnyWheelsConstants.kMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    /** Constructor. */
    public SpinnyIoReal() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.smartCurrentLimit(SpinnyWheelsConstants.kCurrentLimit);
        config.idleMode(IdleMode.kCoast);
        config.openLoopRampRate(0.25);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    }
}
