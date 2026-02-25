package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpinnyWheelsConstants;
import frc.robot.sim.RollerSim.RollerIoInterface;

/** Subsystem for the agitator, which we call spinny. */
public class SpinnyWheels extends SubsystemBase {
    private final RollerIoInterface m_spinnyIO;
    private final RollerIoInterface.DeviceOutputs m_spinnyOutputs =
        new RollerIoInterface.DeviceOutputs();

    /** Constructor. */
    public SpinnyWheels(frc.robot.sim.RollerSim.RollerIoInterface io) {
        m_spinnyIO = io;
    }

    /**
     * Spins the wheels at the speed defined in Constants with a direction provided by the command.
     * 
     * @param direction A double representing direction---shouldn't be greater than 1 or less than -1
     */
    public void spin(double direction) {
        // Gets the sign of direction just in case direction is too big
        m_spinnyIO.setSpeed(Math.signum(direction) * SpinnyWheelsConstants.kSpinSpeed);
    }

    /** Stop. */
    public void stop() {
        m_spinnyIO.stop();
    }

    @Override
    public void periodic() {
        m_spinnyIO.updateOutputs(m_spinnyOutputs);

        // Monitors to ensure the motor is actually spinning
        SmartDashboard.putNumber("Spinny/VelocityRPM", m_spinnyOutputs.velocityRPM);
    }
}
