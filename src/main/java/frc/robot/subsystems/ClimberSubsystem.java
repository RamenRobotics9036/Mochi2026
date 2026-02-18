package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Subsystem for the single-motor climber arm.
 */
public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    // Ramps power over 0.5s to prevent mechanical shock/snapping chains.
    private final SlewRateLimiter m_rampFilter = new SlewRateLimiter(2.0);

    public ClimberSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(ClimberConstants.kCurrentLimit);
        config.idleMode(IdleMode.kBrake); // Holds robot on the chain after match ends
        config.inverted(false); 

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_encoder.setPosition(0);
    }

    /**
     * Sets climber speed with software limit checks.
     */
    public void setClimbSpeed(double request) {
        double filteredSpeed = m_rampFilter.calculate(request);
        double speed = MathUtil.clamp(filteredSpeed, -ClimberConstants.kMaxOutputPercent, ClimberConstants.kMaxOutputPercent);
        
        double currentPos = getEncoderValue();

        // Directional safety: stop if moving toward a limit, allow moving away.
        if (speed > 0 && currentPos >= ClimberConstants.kMaxHeight) {
            speed = 0; 
        } else if (speed < 0 && currentPos <= ClimberConstants.kMinHeight) {
            speed = 0; 
        }

        m_motor.set(speed);
    }

    /** * Manual override that ignores software limits but keeps the safety ramp. 
     */
    public void setClimbSpeedAdmin(double speed) {
        double filteredSpeed = m_rampFilter.calculate(speed);
        m_motor.set(MathUtil.clamp(filteredSpeed, -ClimberConstants.kMaxOutputPercent, ClimberConstants.kMaxOutputPercent));
    }

    public double getEncoderValue() { 
        return m_encoder.getPosition(); 
    }

    public void stop() {
        m_motor.stopMotor();
        m_rampFilter.reset(0); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Position", getEncoderValue());
        SmartDashboard.putNumber("Climber/Amps", m_motor.getOutputCurrent());
        
        // Dashboard status indicators
        SmartDashboard.putBoolean("Climber/At Top", getEncoderValue() >= ClimberConstants.kMaxHeight);
        SmartDashboard.putBoolean("Climber/At Bottom", getEncoderValue() <= ClimberConstants.kMinHeight);
    }
}