package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.sim.RollerSim.RollerIoInterface;
import frc.robot.sim.armsim.ArmIoInterface;

/**
 * Subsystem responsible for the robot's game piece intake mechanism.
 *
 * <p>This subsystem manages intake roller and arm behavior through IO abstractions,
 * allowing the same control logic to run on real hardware and in simulation.
 */
public class IntakeSubsystem extends SubsystemBase {
    private final RollerIoInterface m_intakeIO;
    private final RollerIoInterface.DeviceOutputs m_intakeOutputs =
        new RollerIoInterface.DeviceOutputs();
    

    /**
     * Constructs a new IntakeSubsystem.
     */
    public IntakeSubsystem(RollerIoInterface intakeIO) {
        m_intakeIO = intakeIO;
     }

    /**
     * Sets the intake roller motor output.
     *
     * @param speed Percent output in the range [-1.0, 1.0].
     */
    public void setIntakeSpeed(double speed) {
        m_intakeIO.setSpeed(speed);
        // TODO: check soft limits on arm position
    }

    // Stop all motors in the intake subsystem
    public void stop() {
        m_intakeIO.stop();
    }

    /**
     * Checks if the intake is currently stalled (drawing high current).
     *
     * <p>This is used by commands to detect when a game piece is secured against
     * the rollers or fully inside the mechanism.
     *
     * @return true if the current draw meets or exceeds the threshold in {@link IntakeConstants}.
     */
    public boolean isStalled() {
        // $TODO - Potential bug: The kRollerStallLimit is set to 40 Amps, but the
        // smartCurrentLimit on the motor is also set to 40 Amps.  This means that
        // it is unlikely that isStalled will ever be true.
        // return true if the current draw is above the stall limit
        return m_intakeOutputs.currentAmps >= Constants.IntakeConstants.kIntakeRollerStallLimit;
    }

    /**
     * @return The current draw of the intake motor in Amperes.
     */
    public double getCurrent() {
        return m_intakeOutputs.currentAmps;
    }

    /**
     * Regularly publishes telemetry to the SmartDashboard for driver and pit feedback.
     */
    @Override
    public void periodic() {
        m_intakeIO.updateOutputs(m_intakeOutputs);

        // Publish intake telemetry
        SmartDashboard.putNumber("Intake/VelocityRPM", m_intakeOutputs.velocityRPM);
        SmartDashboard.putNumber("Intake/Current", getCurrent());
        SmartDashboard.putBoolean("Intake/Is Stalled", isStalled());
    }

}
