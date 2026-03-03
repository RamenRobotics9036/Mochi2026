package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.botconfig.BotConfigInterface;
import frc.robot.sim.RollerSim.TwoMotorRollerIoInterface;
import frc.robot.util.LinearServo;

public class ShooterSubsystem extends SubsystemBase {
    private final BotConfigInterface m_configInterface;
    private final TwoMotorRollerIoInterface m_shooterIO;
    private final TwoMotorRollerIoInterface.DeviceOutputs m_shooterOutputs =
        new TwoMotorRollerIoInterface.DeviceOutputs();

    private LinearServo m_hood;

    public ShooterSubsystem(BotConfigInterface configInterface, TwoMotorRollerIoInterface shooterIO) {
        m_configInterface = configInterface;
        m_shooterIO = shooterIO;

        m_hood = new LinearServo(
            Constants.ShooterConstants.kHoodPwmChannel,
            Constants.ShooterConstants.kHoodMaxLength,
            Constants.ShooterConstants.kHoodMaxSpeed
        );
    }

    public void setSpeed(double speed){
        m_shooterIO.setSpeed(speed);
    }

    public void stop(){
        m_shooterIO.stop();
    }

    public void setHoodPosition(double setpoint) {
        m_hood.setLinearPosition(setpoint);
    }

    public void nudgeHoodPosition(double deltaMm) {
        m_hood.setLinearPosition(m_hood.getSetpoint() + deltaMm);
    }

    @Override
    public void periodic() {
        m_shooterIO.updateOutputs(m_shooterOutputs);

        m_hood.updateCurPos();
        m_hood.updateTelemetry();

        SmartDashboard.putNumber("Shooter/VelocityRPM", m_shooterOutputs.velocityRPM);
        SmartDashboard.putNumber("Shooter/HoodCommand_pct", m_hood.getCommandedPercent());
        SmartDashboard.putNumber("Shooter/HoodSetpoint_mm", m_hood.getSetpoint());
        SmartDashboard.putNumber("Shooter/HoodPosition_mm", m_hood.getPosition());
        SmartDashboard.putBoolean("Shooter/HoodFinished", m_hood.isFinished());
    }
}