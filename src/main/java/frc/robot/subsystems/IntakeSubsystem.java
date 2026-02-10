package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Subsystem responsible for the robot's game piece intake mechanism.
 * 
 * <p>This subsystem manages 3 SPARK FLEX motor controllers and provides methods 
 * for running the intake motor and raising/lowering the intake arm, stopping the system
 */
public class IntakeSubsystem extends SubsystemBase {
    /** The motor controller driving the intake rollers. */
    private final SparkFlex m_lArmMotor;
    private final SparkFlex m_rArmMotor;
    private final SparkFlex m_intakeMotor;

    /** The configuration objects applied to the intake motor controller. */
    private final SparkFlexConfig m_lArmConfig;
    private final SparkFlexConfig m_rArmConfig;
    private final SparkFlexConfig m_intakeConfig;

    /**
     * Constructs a new IntakeSubsystem.
     * Configures the motor with brake mode and a smart current limit for safety.
     */
    public IntakeSubsystem() {
        // Left and right arm motors; intake motor:
        m_lArmMotor = new SparkFlex(IntakeConstants.kLeftArmMotorID, MotorType.kBrushless);
        m_rArmMotor = new SparkFlex(IntakeConstants.kRightArmMotorID, MotorType.kBrushless);
        m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        
        // Configure the motors:
        m_lArmConfig = new SparkFlexConfig();
        m_rArmConfig = new SparkFlexConfig();
        m_intakeConfig = new SparkFlexConfig();

        m_lArmConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kStallLimit);
        m_rArmConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kStallLimit)
            .inverted(true)
            .follow(m_lArmMotor);
        m_intakeConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.kStallLimit);

        // Apply configs to controllers (matches pattern used in ShooterSubsystem)
        m_lArmMotor.configure(m_lArmConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        m_rArmMotor.configure(m_rArmConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        m_intakeMotor.configure(m_intakeConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    /**
     * Sets the open-loop percent output for the intake arm motors.
     *
     * <p>Positive values should correspond to "deploy" and negative values to "raise",
     * but the sign convention ultimately depends on motor inversion/mechanics.
     *
     * @param speed Percent output in the range [-1.0, 1.0].
     */
    public void setArmSpeed(double speed) {
        m_lArmMotor.set(speed);
        // Right arm follows left via configuration, but set it anyway for clarity/safety.
        m_rArmMotor.set(speed);
    }

    /** Convenience wrapper for setting an explicit deploy (down) speed. */
    public void setDeployArmSpeed(double speed) {
        setArmSpeed(speed);
    }

    /** Convenience wrapper for setting an explicit raise (up) speed. */
    public void setRaiseArmSpeed(double speed) {
        setArmSpeed(speed);
    }

    // 
    public boolean isArmDeployed() {
        // todo
        return false;
    }

    /**
     * Sets the intake roller motor output.
     *
     * @param speed Percent output in the range [-1.0, 1.0].
     */
    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    // Cut power to the arm motors
    public void stopArm() {
        // todo: reset mode to idle
        m_lArmMotor.stopMotor();
        m_rArmMotor.stopMotor();
    }

    // Immediately cuts power to the intake motor
    public void stopIntake() {
        // todo: reset mode to idle
        m_intakeMotor.stopMotor();
    }

    // Stop all motors in the intake subsystem
    public void stop() {
        stopIntake();
        stopArm();
    }

    /**
     * Checks if the intake is currently stalled (drawing high current).
     * 
     * <p>This is used by commands to detect when a game piece is secured against 
     * the rollers or fully inside the mechanism.
     * 
     * @return true if the current draw meets or exceeds the threshold in {@link IntakeConstants}.
     */
    public boolean isStalled() {
        // return true if the current draw is above the stall limit
        return m_intakeMotor.getOutputCurrent() >= IntakeConstants.kStallLimit;

    }

    /**
     * @return The current draw of the intake motor in Amperes.
     */
    public double getCurrent() {
        return m_intakeMotor.getOutputCurrent();
    }

    /**
     * Regularly publishes telemetry to the SmartDashboard for driver and pit feedback.
     */
    @Override
    public void periodic() {
        // Publish intake telemetry
        SmartDashboard.putNumber("Intake/Current", getCurrent());
        SmartDashboard.putBoolean("Intake/Is Stalled", isStalled());
    }
}
