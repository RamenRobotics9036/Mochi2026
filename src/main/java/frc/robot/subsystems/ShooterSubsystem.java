package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem {
    private TalonFX m_lMotor;
    private TalonFX m_rMotor;
    private Servo m_hood;

    public ShooterSubsystem(){
        m_lMotor = new TalonFX(ShooterConstants.kLMotorID);
        m_rMotor = new TalonFX(ShooterConstants.kRMotorID);
        m_hood = new Servo(ShooterConstants.kChannel);
    }

    public void SetSpeed(double speed){
        m_lMotor.set(speed);
        //Facing the opposite direction.
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
