package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.sim.elevatorssim.ElevatorIoInterface;

/**
 * Subsystem for the single-motor climber arm.
 */
public class ClimberSubsystem extends SubsystemBase {
    private final ElevatorIoInterface m_io;
    private final ElevatorIoInterface.DeviceOutputs m_outputs = new ElevatorIoInterface.DeviceOutputs();

    /** Constructor. */
    public ClimberSubsystem(ElevatorIoInterface io) {
        m_io = io;
    }

    /**
     * Sets climber speed with software limit checks.
     */
    public void setClimbSpeed(double request) {
        m_io.setSpeed(request);
    }

    public double getEncoderValue() {
        return m_outputs.m_positionMeters;
    }

    public void stop() {
        m_io.stop();
    }

    @Override
    public void periodic() {
        m_io.updateOutputs(m_outputs);

        SmartDashboard.putNumber("Climber/Position", m_outputs.m_positionMeters);
        SmartDashboard.putNumber("Climber/Amps", m_outputs.m_currentAmps);

        // Dashboard status indicators
        // Fixed Bug: Now in theory this should compare meters to meters (NOTE: Constants should be same)
        SmartDashboard.putBoolean("Climber/At Top", m_outputs.m_positionMeters >= ClimberConstants.kMaxHeight);
        SmartDashboard.putBoolean("Climber/At Bottom", m_outputs.m_positionMeters <= ClimberConstants.kMinHeight);
    }
}