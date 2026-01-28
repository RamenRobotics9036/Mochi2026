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
 * Subsystem responsible for the robot's game piece intake mechanism.
 * 
 * <p>This subsystem manages a single SPARK MAX motor controller and provides methods 
 * for running the intake at specific speeds, stopping the system, and monitoring 
 * motor current to detect when a game piece has been successfully acquired.
 */
public class IntakeSubsystem extends SubsystemBase {
    /** The motor controller driving the intake rollers. */
    private final SparkMax m_intakeMotor;
    /** The configuration object applied to the intake motor controller. */
    private final SparkMaxConfig m_config;

    /**
     * Constructs a new IntakeSubsystem.
     * Configures the motor with brake mode and a smart current limit for safety.
     */
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
     * 
     * @param speed Percent output from -1.0 to 1.0 (positive is intake, negative is outtake).
     */
    public void setSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    /**
     * Immediately cuts power to the intake motor.
     */
    public void stop() {
        // stop the motor
        m_intakeMotor.stopMotor();
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
