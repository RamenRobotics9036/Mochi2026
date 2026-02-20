package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
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
    public ClimberSubsystem() {
    }

    /**
     * Sets climber speed with software limit checks.
     */
    public void setClimbSpeed(double request) {

    }

    public void setClimbSpeedAdmin(double speed) {
        double filteredSpeed = m_rampFilter.calculate(speed);
        m_motor.set(MathUtil.clamp(filteredSpeed, -ClimberConstants.kMaxOutputPercent, ClimberConstants.kMaxOutputPercent));
    }

    public void stop() {
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