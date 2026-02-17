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

    private TalonFX m_lMotor;
    private TalonFX m_rMotor;

    private TalonFXConfiguration m_lConfig;
    private TalonFXConfiguration m_rConfig;

    private Servo m_hood;

    /** Creates a new ShooterSubsystem using the ShooterConstants from Constants.java. */
    public ShooterSubsystem(BotConfigInterface configInterface){
        m_configInterface = configInterface;

        m_lMotor = new TalonFX(Constants.ShooterConstants.kLMotorID, m_configInterface.getCANBus());
        m_rMotor = new TalonFX(Constants.ShooterConstants.kRMotorID, m_configInterface.getCANBus());

        m_lConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kCurrentStatorLimit)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kCurrentSupplyLimit)
            );
        m_rConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    // Previously, we set this motor to be a follower and inverted it here
                    // However, being a follower isn't part of the configuration for Kraken motors
                    // This is instead done later, and inversion is done there too due to how following works
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.ShooterConstants.kCurrentStatorLimit)
                    .withSupplyCurrentLimit(Constants.ShooterConstants.kCurrentSupplyLimit)
            );

        m_lMotor.getConfigurator().apply(m_lConfig);
        m_rMotor.getConfigurator().apply(m_rConfig);

        // Sets the right motor to follow the left one
        // Also sets its direction to be opposed rather than inverting it earlier in the code
        m_rMotor.setControl(new Follower(Constants.ShooterConstants.kLMotorID, MotorAlignmentValue.Opposed));

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
