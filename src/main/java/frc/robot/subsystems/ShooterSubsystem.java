package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem {
    private TalonFX m_lMotor;
    private TalonFX m_rMotor;

    public ShooterSubsystem(){
        m_lMotor = new TalonFX(ShooterConstants.kLMotorID);
        m_rMotor = new TalonFX(ShooterConstants.kRMotorID);
    }

    public void SetSpeed(double speed){
        m_lMotor.set(speed);
        //Inverted. Facing the opposite direction.
        m_rMotor.set(-speed);
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
