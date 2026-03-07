package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sim.RollerSim.TwoMotorRollerIoInterface;

public class ShooterSubsystem extends SubsystemBase {
    private final TwoMotorRollerIoInterface m_shooterIO;
    private final TwoMotorRollerIoInterface.DeviceOutputs m_shooterOutputs =
        new TwoMotorRollerIoInterface.DeviceOutputs();

    public ShooterSubsystem(TwoMotorRollerIoInterface shooterIO) {
        m_shooterIO = shooterIO;
    }

    public void setSpeed(double speed){
        m_shooterIO.setSpeed(speed);
    }

    public void stop(){
        m_shooterIO.stop();
    }

    @Override
    public void periodic() {
        m_shooterIO.updateOutputs(m_shooterOutputs);

        SmartDashboard.putNumber("Shooter/VelocityRPM", m_shooterOutputs.velocityRPM);
    }
}
