package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
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
 * <p>This subsystem manages 3 SPARK FLEX motor controllers and provides methods 
 * for running the intake motor and raising/lowering the intake arm, stopping the system
 */
public class IntakeSubsystem extends SubsystemBase {
    /** The motor controller driving the intake rollers. */
    private final SparkFlex m_lArmMotor;
    private final SparkFlex m_rArmMotor;
    private final SparkFlex m_intakeMotor;
    /** The configuration object applied to the intake motor controller. */
    //private final SparkMaxConfig m_config;

    /**
     * Constructs a new IntakeSubsystem.
     * Configures the motor with brake mode and a smart current limit for safety.
     */
    public IntakeSubsystem() {
        m_lArmMotor = new SparkFlex(IntakeConstants.kLeftArmMotorID);
        m_rArmMotor = new SparkFlex(IntakeConstants.kRightArmMotorID);
        m_intakeMotor = new SparkFlex(IntakeConstants.kIntakeMotorID);
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
