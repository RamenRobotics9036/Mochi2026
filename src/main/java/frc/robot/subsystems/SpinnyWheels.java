package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpinnyWheelsConstants;
import robotutils.pub.interfaces.RollerIoInterface;


/** Subsystem for the agitator, which we call spinny. */
public class SpinnyWheels extends SubsystemBase {
    private final RollerIoInterface m_spinnyIO;
    private final RollerIoInterface.DeviceOutputs m_spinnyOutputs =
        new RollerIoInterface.DeviceOutputs();

    /** Constructor. */
    public SpinnyWheels(RollerIoInterface io) {
        m_spinnyIO = io;
    }

    /**
     * Spins the wheels at the speed defined in Constants.
     */
    public void spinCounterclockwise() {
        m_spinnyIO.setSpeed(SpinnyWheelsConstants.kSpinSpeed);
    }

    /**
     * Spins the wheels backwards at the speed defined in Constants.
     */
    public void spinClockwise() {
        m_spinnyIO.setSpeed(-SpinnyWheelsConstants.kSpinSpeed);
    }

    /** Stop. */
    public void stop() {
        m_spinnyIO.stop();
    }

    @Override
    public void periodic() {
        m_spinnyIO.updateOutputs(m_spinnyOutputs);

        // Monitors to ensure the motor is actually spinning
        SmartDashboard.putNumber("Spinny/VelocityRPM", m_spinnyOutputs.m_velocityRpm);
    }
}
