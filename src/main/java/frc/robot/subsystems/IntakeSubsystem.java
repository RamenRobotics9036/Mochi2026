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
    private final ArmIoInterface m_armIO;
    private final ArmIoInterface.DeviceOutputs m_armOutputs =
        new ArmIoInterface.DeviceOutputs();

    // Enum state to track if the arm has homed
    private enum ArmHomedState {
        HOMED,
        HOMING,
        NOT_HOMED
    }
    private ArmHomedState m_HomingState = ArmHomedState.NOT_HOMED;

    // Homing bookkeeping
    private final Timer m_homingTimer = new Timer();
    private double m_stallStartSec = -1.0;

    /**
     * Constructs a new IntakeSubsystem.
     */
    public IntakeSubsystem(
            RollerIoInterface intakeIO,
            ArmIoInterface armIO) {
        m_intakeIO = intakeIO;
        m_armIO = armIO;
     }

    /**
     * Immediately starts moving arm.
     *
     * <p>Positive values should correspond to "deploy" and negative values to "raise",
     * but the sign convention ultimately depends on motor inversion/mechanics.
     *
     * @param speed Percent output in the range [-1.0, 1.0].
     */
    public void moveArmWithSpeed(double speed) {
        m_armIO.moveArmWithSpeed(speed);
    }

    // Homes the intake arm by moving it to the zero position
    public void beginHoming() {
        if(m_HomingState != ArmHomedState.HOMING) {
            m_HomingState = ArmHomedState.HOMING;
            m_homingTimer.restart();
            m_stallStartSec = -1.0;
        }
    }

    public void setArmPosition(double position) {
        m_armIO.setPosition(
            MathUtil.clamp(position, IntakeConstants.kMinArmAngle, IntakeConstants.kMaxArmAngle));
    }

    //
    public boolean isArmDeployed() {
        return m_armOutputs.position >= (IntakeConstants.kMaxArmAngle - 1.0);
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

    // Cut power to the arm motors
    public void stopArm() {
        m_armIO.stop();
    }

    // Immediately cuts power to the intake motor
    public void stopIntake() {
        // todo: reset mode to idle
        m_intakeIO.stop();
    }

    // Stop all motors in the intake subsystem
    public void stop() {
        stopIntake();
        stopArm();
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
        m_armIO.updateOutputs(m_armOutputs);

        // Publish intake telemetry
        SmartDashboard.putNumber("Intake/VelocityRPM", m_intakeOutputs.velocityRPM);
        SmartDashboard.putNumber("Intake/Current", getCurrent());
        SmartDashboard.putBoolean("Intake/Is Stalled", isStalled());
        SmartDashboard.putString("Intake/ArmHomingState", m_HomingState.name());
        SmartDashboard.putNumber("Intake/ArmPosition", m_armOutputs.position);
        SmartDashboard.putNumber("Intake/ArmVelocity", m_armOutputs.velocity);
        if(m_HomingState == ArmHomedState.HOMING) {
            homingHelper();
        }
    }

    public void homingHelper() {
        if (m_HomingState != ArmHomedState.HOMING) {
            return;
        }

        // Creep toward the hard stop. You may need to flip this sign once the mechanism is on the robot.
        m_armIO.moveArmWithSpeed(IntakeConstants.kArmHomingSpeed);

        final double current = m_armOutputs.currentAmps;
        final double velocity = Math.abs(m_armOutputs.velocity);
        final boolean looksStalled = current >= IntakeConstants.kArmHomingStallCurrent
                && velocity <= IntakeConstants.kArmHomingStallVelocity;

        if (looksStalled) {
            if (m_stallStartSec < 0.0) {
                m_stallStartSec = m_homingTimer.get();
            }
        } else {
            m_stallStartSec = -1.0;
        }

        final boolean stallLongEnough = m_stallStartSec >= 0.0
                && (m_homingTimer.get() - m_stallStartSec) >= IntakeConstants.kArmHomingStallSeconds;

        if (stallLongEnough) {
            stopArm();
            // This stop is the mechanical reference (0). Adjust if your "zero" should be some other angle.
            m_armIO.resetEncoderValue();
            m_HomingState = ArmHomedState.HOMED;
            m_homingTimer.stop();
            return;
        }

        if (m_homingTimer.get() >= IntakeConstants.kArmHomingTimeoutSeconds) {
            // Timeout: stop and mark as NOT_HOMED so higher-level code can decide what to do.
            stopArm();
            m_HomingState = ArmHomedState.NOT_HOMED;
            m_homingTimer.stop();
        }
    }
}
