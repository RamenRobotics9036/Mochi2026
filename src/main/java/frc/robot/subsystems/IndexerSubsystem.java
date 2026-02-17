package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;


public class IndexerSubsystem extends SubsystemBase{
    private SparkFlex m_motor;
    private SparkFlexConfig m_config;

    public IndexerSubsystem(){
        m_motor = new SparkFlex(IndexerConstants.kMotorID, MotorType.kBrushless);

        m_config = new SparkFlexConfig();

        m_config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IndexerConstants.kIndexerCurrentLimit)
            .inverted(true);

        m_motor.configure(m_config, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed){
        m_motor.set(speed);
    }

    public double getSpeed(){
        return m_motor.get();
    }

    public void stop(){
        m_motor.stopMotor();
    }
}