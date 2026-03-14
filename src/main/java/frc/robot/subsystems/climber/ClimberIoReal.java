package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.ClimberConstants;
import frc.robot.sim.elevatorssim.ElevatorIoInterface;

/**
 * Real-hardware implementation of {@link ElevatorIoInterface} for the climber.
 *
 * <p>Methods are intentionally left blank for now; the real motor
 * control still lives in {@code ClimberSubsystem} until migration
 * is complete.
 */
public class ClimberIoReal implements ElevatorIoInterface {
    private final SparkFlex m_motor;
    private final RelativeEncoder m_encoder;
    private final SlewRateLimiter m_rampFilter;

    /** Constructor. */
    public ClimberIoReal() {
        m_motor = new SparkFlex(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();

        // Ramps power over 0.5s to prevent mechanical shock/snapping chains.
        m_rampFilter = new SlewRateLimiter(ClimberConstants.kClimbSlewRate);

        SparkFlexConfig config = new SparkFlexConfig();

        config.smartCurrentLimit(ClimberConstants.kCurrentLimit);
        config.idleMode(IdleMode.kBrake); // Holds robot on the chain after match ends
        config.inverted(false);

        // Apply configuration
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // $TODO - Bug: What happens if the robot is powered on while the climber is extended?
        // Will the encoder here think it's at 0 and therefore allow movement further up, risking damage?
        m_encoder.setPosition(0);
    }

    @Override
    public void setSpeed(double speed) {
        double filteredSpeed = m_rampFilter.calculate(speed);
        double clampedSpeed = MathUtil.clamp(
            filteredSpeed,
            -ClimberConstants.kMaxOutputPercent,
            ClimberConstants.kMaxOutputPercent);

        double currentPos = getEncoderValue();

        // Directional safety: stop if moving toward a limit, allow moving away.
        //if (clampedSpeed > 0 && currentPos >= ClimberConstants.kMaxHeight) {
          //  clampedSpeed = 0;
        //} else if (clampedSpeed < 0 && currentPos <= ClimberConstants.kMinHeight) {
          //  clampedSpeed = 0;
        //}

        m_motor.set(clampedSpeed);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
        m_rampFilter.reset(0);
    }

    @Override
    public void updateOutputs(ElevatorIoInterface.DeviceOutputs outputs) {
        outputs.m_positionMeters = m_encoder.getPosition();
        outputs.m_currentAmps = m_motor.getOutputCurrent();
    }

    private double getEncoderValue() {
        return m_encoder.getPosition();
    }
}
