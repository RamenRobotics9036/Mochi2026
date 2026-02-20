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

        // $TODO - Bug?  The units for encoder (after conversion here) seem to be rotations/second
        // I think?  But setPosition below takes as input DEGREES I think?  We should check all the units
        // in the arm code, and change variable. names to make clear what the units are: e.g. positionDegrees, etc.
        lArmConfig.encoder
            .positionConversionFactor(1.0 / IntakeConstants.kArmGearRatio)
            .velocityConversionFactor((1.0 / IntakeConstants.kArmGearRatio) / 60.0);

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
        // $TODO - Previously, IntakeSubsystem.setArmSpeed just set speed on m_lArmMotor.
        // But IntakeSubsystem.homingHelper set kArmHomingSpeed on BOTH m_lArmMotor AND
        // m_rArmMotor.  Now, homingHelper calls this same method, which only sets speed on m_lArmMotor.
        // Is that safe, or will it break the arm?
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
