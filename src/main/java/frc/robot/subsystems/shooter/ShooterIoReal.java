package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.sim.RollerSim.TwoMotorRollerIoInterface;

/**
 * Real-hardware implementation of {@link TwoMotorRollerIoInterface} for the shooter.
 *
 * <p>Uses two TalonFX motors: the left motor is the leader and the right motor
 * follows it with {@link MotorAlignmentValue#Opposed} so they spin the flywheel
 * in the same physical direction.
 */
public class ShooterIoReal implements TwoMotorRollerIoInterface {
    private BotConfigInterface m_configInterface;

    private final TalonFX m_lMotor;
    private final TalonFX m_rMotor;

    /** Constructs the real shooter IO and configures both motors. */
    public ShooterIoReal(BotConfigInterface configInterface) {
        TalonFXConfiguration lConfig;
        TalonFXConfiguration rConfig;

        m_configInterface = configInterface;

        m_lMotor = new TalonFX(Constants.ShooterConstants.kLMotorID, m_configInterface.getCANBus());
        m_rMotor = new TalonFX(Constants.ShooterConstants.kRMotorID, m_configInterface.getCANBus());

        lConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kCurrentStatorLimit)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kCurrentSupplyLimit)
            );
        rConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    // Previously, we set this motor to be a follower and inverted it here
                    // However, being a follower isn't part of the configuration for Kraken motors
                    // This is instead done later, and inversion is done there too due to how following works
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kCurrentStatorLimit)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kCurrentSupplyLimit)
            );

        m_lMotor.getConfigurator().apply(lConfig);
        m_rMotor.getConfigurator().apply(rConfig);

        // Sets the right motor to follow the left one
        // Also sets its direction to be opposed rather than inverting it earlier in the code
        m_rMotor.setControl(new Follower(Constants.ShooterConstants.kLMotorID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void setSpeed(double speed) {
        // NOTE: The right motor will automatically follow the left motor speed.
        m_lMotor.set(speed);
    }

    @Override
    public void stop() {
        // $TODO - SAFETY: We should stop m_rMotor as well, since its possible for the follower relationship
        // to get interrupted, in which case the right motor could keep spinning, and break the shooter.

        m_lMotor.stopMotor();
    }

    @Override
    public void updateOutputs(DeviceOutputs outputs) {
        outputs.velocityRPM = m_lMotor.getVelocity().getValueAsDouble() * 60.0;
        outputs.leaderCurrentAmps = m_lMotor.getStatorCurrent().getValueAsDouble();
        outputs.followerCurrentAmps = m_rMotor.getStatorCurrent().getValueAsDouble();
    }
}
