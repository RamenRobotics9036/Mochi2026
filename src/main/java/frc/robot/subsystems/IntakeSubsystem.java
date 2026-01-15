package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * IntakeSubsystem manages the robot's intake motor.
 * It provides basic control for intaking and outtaking game pieces,
 * with current-based stall detection for automatic stopping.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax m_intakeMotor;
    private final SparkMaxConfig m_config;

    public IntakeSubsystem() {
        m_intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        m_config = new SparkMaxConfig();

        // Basic motor configuration
        m_config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.kStallLimit);
        
        m_intakeMotor.configure(m_config, 
            SparkBase.ResetMode.kResetSafeParameters, 
            SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Sets the intake motor speed.
     * @param speed Speed from -1.0 to 1.0 (positive is intake, negative is outtake)
     */
    public void setSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        m_intakeMotor.stopMotor();
    }

    /**
     * Checks if the intake is currently stalled (drawing high current).
     * This is typically used to detect when a game piece has been fully intaked.
     * @return true if motor current exceeds the stall limit
     */
    public boolean isStalled() {
        return m_intakeMotor.getOutputCurrent() >= IntakeConstants.kStallLimit;
    }

    /**
     * @return The current draw of the intake motor in Amperes.
     */
    public double getCurrent() {
        return m_intakeMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        // Log telemetry to SmartDashboard
        SmartDashboard.putNumber("Intake/Current", getCurrent());
        SmartDashboard.putBoolean("Intake/Is Stalled", isStalled());
    }
}
