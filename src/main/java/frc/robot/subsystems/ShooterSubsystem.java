package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private SparkFlex m_lMotor;
    private SparkFlex m_rMotor;

    private SparkFlexConfig m_lConfig;
    private SparkFlexConfig m_rConfig;

    private Servo m_hood;

    public ShooterSubsystem(){
        m_lMotor = new SparkFlex(ShooterConstants.kLMotorID, MotorType.kBrushless);
        m_rMotor = new SparkFlex(ShooterConstants.kRMotorID, MotorType.kBrushless);

        m_lConfig = new SparkFlexConfig();
        m_rConfig = new SparkFlexConfig();

        m_lConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.kCurrentLimit);

        m_rConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.kCurrentLimit)
            .inverted(true)
            .follow(m_lMotor);

        m_hood = new Servo(ShooterConstants.kChannel);
    }

    public void SetSpeed(double speed){
        m_lMotor.set(speed);
    }
    
    //Gets the speed of the uninverted left motor
    public double getSpeed(){
        return m_lMotor.get();
    }

    public void stop(){
        m_lMotor.stopMotor();
        m_rMotor.stopMotor();
    }
}
