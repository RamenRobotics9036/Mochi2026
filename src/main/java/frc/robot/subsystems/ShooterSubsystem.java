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
import frc.robot.sim.RollerSim.TwoMotorRollerIoInterface;

public class ShooterSubsystem extends SubsystemBase {
    private final BotConfigInterface m_configInterface;
    private final TwoMotorRollerIoInterface m_shooterIO;

    private Servo m_hood;

    /** Creates a new ShooterSubsystem using the ShooterConstants from Constants.java. */
    public ShooterSubsystem(BotConfigInterface configInterface, TwoMotorRollerIoInterface shooterIO) {
        m_configInterface = configInterface;
        m_shooterIO = shooterIO;

        m_hood = new Servo(Constants.ShooterConstants.kHoodPwmChannel);
    }

    public void setSpeed(double speed){

        m_shooterIO.setSpeed(speed);
    }

    public void stop(){
        m_shooterIO.stop();

    }
}
