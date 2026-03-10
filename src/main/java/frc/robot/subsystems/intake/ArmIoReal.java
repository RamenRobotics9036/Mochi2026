package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.ArmConstants;
import robotutils.pub.interfaces.ArmIoInterface;


/**
 * Real-hardware implementation of {@link ArmIoInterface}.
 */
public class ArmIoReal implements ArmIoInterface {
    private final SparkFlex m_lArmMotor;
    private final SparkFlex m_rArmMotor;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_pidController;

    /** Constructor. */
    public ArmIoReal() {
        SparkFlexConfig lArmConfig = new SparkFlexConfig();
        SparkFlexConfig rArmConfig = new SparkFlexConfig();

        m_lArmMotor = new SparkFlex(ArmConstants.kLeftArmMotorID, MotorType.kBrushless);
        m_rArmMotor = new SparkFlex(ArmConstants.kRightArmMotorID, MotorType.kBrushless);

        // Configure the motors:
        lArmConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.kArmStallLimit);

        // Convert motor rotations → arm degrees.
        // Motor shaft rotates kArmGearRatio times per one output rotation (360°),
        // so dividing by kArmGearRatio gives output rotations; multiplying by 360 gives degrees.
        // Velocity: raw units are motor RPM; dividing by 60 converts to motor RPS,
        // then dividing by kArmGearRatio gives output RPS; multiplying by 360 gives deg/s.
        lArmConfig.encoder
            .positionConversionFactor(360.0 / ArmConstants.kArmGearRatio)
            .velocityConversionFactor(360.0 / ArmConstants.kArmGearRatio / 60.0);

        lArmConfig.closedLoop
            .p(ArmConstants.kArmPositionP);

        // Soft limits protect against overshoot in closed-loop mode.
        // They are defined here but start DISABLED because the encoder is not zeroed until
        // homing completes. Call setSoftLimitsEnabled(true) after homing.
        // $TODO verify unit assumption: REV applies soft limits against the converted encoder
        // value (degrees), so kMaxArmAngle/kMinArmAngle should be correct here.
        lArmConfig.softLimit
            .forwardSoftLimit((float) ArmConstants.kMaxArmAngle)
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit((float) ArmConstants.kMinArmAngle)
            .reverseSoftLimitEnabled(false);

        rArmConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ArmConstants.kArmStallLimit)
            .follow(m_lArmMotor, true);

        // Apply configs to controllers (matches pattern used in ShooterSubsystem)
        m_lArmMotor.configure(lArmConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        m_rArmMotor.configure(rArmConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        // Initialize the encoder and PID controller for the arm motors
        m_encoder = m_lArmMotor.getEncoder();
        m_pidController = m_lArmMotor.getClosedLoopController();
    }

    @Override
    public void moveArmWithSpeed(double speed) {
        // Only the left motor needs to be commanded. The right motor is configured with
        // .follow(m_lArmMotor, true) above, so the SparkFlex hardware automatically mirrors
        // the left motor's output (inverted). No separate call to m_rArmMotor is needed.
        m_lArmMotor.set(speed);
    }

    @Override
    public void setPosition(double position) {
        m_pidController.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void resetEncoderValue() {
        m_encoder.setPosition(0.0);
    }

    @Override
    public void stop() {
        m_lArmMotor.stopMotor();
        m_rArmMotor.stopMotor();
    }

    @Override
    public void setSoftLimitsEnabled(boolean enabled) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.softLimit
            .forwardSoftLimitEnabled(enabled)
            .reverseSoftLimitEnabled(enabled);
        m_lArmMotor.configure(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
    }

    @Override
    public void updateOutputs(DeviceOutputs outputs) {
        outputs.position = m_encoder.getPosition();
        outputs.velocity = m_encoder.getVelocity();
        outputs.currentAmps = m_lArmMotor.getOutputCurrent();
    }
}
