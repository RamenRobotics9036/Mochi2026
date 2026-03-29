package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotutils.pub.interfaces.RollerIoInterface;


/** Indexer subsystem. */
public class IndexerSubsystem extends SubsystemBase {
    private final RollerIoInterface m_indexerIO;
    private final RollerIoInterface.DeviceOutputs m_indexerOutputs =
        new RollerIoInterface.DeviceOutputs();

    /** Constructor. */
    public IndexerSubsystem(RollerIoInterface indexerIO) {
        m_indexerIO = indexerIO;
    }

    /** Set the motor speed. */
    public void setSpeed(double speed) {
        m_indexerIO.setSpeed(speed);
    }

    /** Stop motor. */
    public void stop() {
        m_indexerIO.stop();
    }

    @Override
    public void periodic() {
        m_indexerIO.updateOutputs(m_indexerOutputs);

        SmartDashboard.putNumber("Indexer/VelocityRPM", m_indexerOutputs.m_velocityRpm);
    }
}
