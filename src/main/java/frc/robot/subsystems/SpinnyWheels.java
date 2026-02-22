package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpinnyWheelsConstants; 

public class SpinnyWheels extends SubsystemBase {
    private final SparkFlex m_motor = new SparkFlex(SpinnyWheelsConstants.kMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();

    public SpinnyWheels(frc.robot.sim.RollerSim.RollerIoInterface io) {
        SparkFlexConfig config = new SparkFlexConfig();

        config.smartCurrentLimit(SpinnyWheelsConstants.kCurrentLimit);
        config.idleMode(IdleMode.kCoast); 
        config.openLoopRampRate(0.25); 

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Spins the wheels at the constant speed defined in Constants.
     */
    public void spin() {
        m_motor.set(SpinnyWheelsConstants.kSpinSpeed);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void periodic() {
        // Monitors to ensure the motor is actually spinning
        SmartDashboard.putNumber("Spinny/Current RPM", m_encoder.getVelocity());
    }
}