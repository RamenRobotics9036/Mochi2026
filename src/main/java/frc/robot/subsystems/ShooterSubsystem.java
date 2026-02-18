package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.botconfig.BotConfigInterface;

public class ShooterSubsystem extends SubsystemBase {
    private BotConfigInterface m_configInterface;

    private TalonFX m_lMotor; // $TODO - Should go away
    private TalonFX m_rMotor; // $TODO - Should go away

    private TalonFXConfiguration m_lConfig; // $TODO - Should go away
    private TalonFXConfiguration m_rConfig; // $TODO - Should go away

    private Servo m_hood;

    /** Creates a new ShooterSubsystem using the ShooterConstants from Constants.java. */
    public ShooterSubsystem(BotConfigInterface configInterface){
        m_hood = new Servo(Constants.ShooterConstants.kHoodPwmChannel);
    }

    public void setSpeed(double speed){
        // $TODO: Use PID to ensure the motors are spinning at exactly the right speed.
        m_lMotor.set(speed);
    }

    //Gets the speed of the uninverted left motor
    public double getSpeed(){
        return m_lMotor.get();
    }

    public void stop(){
        m_lMotor.stopMotor();

        // $TODO - SAFETY: We should stop m_rMotor as well, since its possible for the follower relationship
        // to get interrupted, in which case the right motor could keep spinning, and break the shooter.
    }
}
