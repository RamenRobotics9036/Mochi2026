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
import frc.robot.Constants.IntakeConstants;
import frc.robot.sim.armsim.ArmIoInterface;

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

        m_lArmMotor = new SparkFlex(IntakeConstants.kLeftArmMotorID, MotorType.kBrushless);
        m_rArmMotor = new SparkFlex(IntakeConstants.kRightArmMotorID, MotorType.kBrushless);

        // Configure the motors:
        lArmConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kArmStallLimit);
        lArmConfig.encoder
            .positionConversionFactor(1.0 / IntakeConstants.kArmGearRatio) // $TODO - 360.0 / IntakeConstants.kArmGearRatio?
            .velocityConversionFactor((1.0 / IntakeConstants.kArmGearRatio) / 60.0); // $TODO - (360.0 / IntakeConstants.kArmGearRatio) / 60.0?

        rArmConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kArmStallLimit)
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
        m_lArmMotor.set(speed);

        // $TODO - Is this right to set the right-arm speed when its a follower of left arm?  This may
        // break the follower relationship, and cause the two motors to move independently?  We should
        // verify and potentially fix.
        m_rArmMotor.set(IntakeConstants.kArmHomingSpeed);
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
       // todo: reset mode to idle
        m_lArmMotor.stopMotor();
        m_rArmMotor.stopMotor();
    }

    @Override
    public void updateOutputs(DeviceOutputs outputs) {
        outputs.position = m_encoder.getPosition();
        outputs.velocity = m_encoder.getVelocity();
        outputs.currentAmps = m_lArmMotor.getOutputCurrent();
    }
}
