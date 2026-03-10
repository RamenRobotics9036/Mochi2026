package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import robotutils.pub.interfaces.simio.ArmIoInterface;

public class ArmSubsystem extends SubsystemBase{
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

    public ArmSubsystem(ArmIoInterface armIO){
        m_armIO = armIO;
    }

    /**
     * Regularly publishes telemetry to the SmartDashboard for driver and pit feedback.
     */
    @Override
    public void periodic() {
        m_armIO.updateOutputs(m_armOutputs);

        //Publish arm telemetry
        SmartDashboard.putString("Intake/ArmHomingState", m_HomingState.name());
        SmartDashboard.putNumber("Intake/ArmPosition", m_armOutputs.position);
        SmartDashboard.putNumber("Intake/ArmVelocity", m_armOutputs.velocity);
        SmartDashboard.putBoolean("Intake/ArmAtMin", m_armOutputs.position <= ArmConstants.kMinArmAngle);
        SmartDashboard.putBoolean("Intake/ArmAtMax", m_armOutputs.position >= ArmConstants.kMaxArmAngle);
        if(m_HomingState == ArmHomedState.HOMING) {
            homingHelper();
        }
    }

    /**
     * Immediately starts moving arm.
     *
     * <p>Positive values correspond to "deploy" and negative values to "raise".
     * If homing is in progress, calling this method aborts homing and marks the arm NOT_HOMED.
     * Once homed, soft limits prevent commanding past {@code kMinArmAngle} or {@code kMaxArmAngle}.
     *
     * @param speed Percent output in the range [-1.0, 1.0].
     */
    public void moveArmWithSpeed(double speed) {
        // Abort homing if operator manually commands the arm
        if (m_HomingState == ArmHomedState.HOMING) {
            m_armIO.stop();
            m_HomingState = ArmHomedState.NOT_HOMED;
            m_homingTimer.stop();
        }

        // Soft limits: only enforce when position is known (homed)
        if (m_HomingState == ArmHomedState.HOMED) {
            boolean atMax = m_armOutputs.position >= ArmConstants.kMaxArmAngle;
            boolean atMin = m_armOutputs.position <= ArmConstants.kMinArmAngle;
            if (speed > 0 && atMax) speed = 0;
            if (speed < 0 && atMin) speed = 0;
        }

        m_armIO.moveArmWithSpeed(speed);
    }

    /** Homes the arm by moving it to the zero position. */
    public void beginHoming() {
        if(m_HomingState != ArmHomedState.HOMING) {
            m_armIO.setSoftLimitsEnabled(false);
            m_HomingState = ArmHomedState.HOMING;
            m_homingTimer.restart();
            m_stallStartSec = -1.0;
        }
    }

    /**
     * Sets the desired position for the arm in degrees.
     *
     * <p>Ignored (with a dashboard warning) until the arm has been homed, because the relative
     * encoder has no valid zero reference until homing completes. The position is clamped to
     * [{@code kMinArmAngle}, {@code kMaxArmAngle}] before being sent to the controller.
     *
     * @param position Target arm angle in degrees.
     */
    public void setArmPosition(double position) {
        if (m_HomingState != ArmHomedState.HOMED) {
            DriverStation.reportWarning("ArmSubsystem: setArmPosition called before homing — ignored", false);
            SmartDashboard.putString("Intake/ArmWarning", "setArmPosition before homing — ignored");
            return;
        }
        m_armIO.setPosition(
            MathUtil.clamp(position, ArmConstants.kMinArmAngle, ArmConstants.kMaxArmAngle));
    }

    /** Manually resets the arm encoder to zero. */
    public void setArmZero() {
        m_armIO.resetEncoderValue();
    }

    /** Checks if the arm is deployed. */
    public boolean isArmDeployed() {
        return m_armOutputs.position >= (ArmConstants.kMaxArmAngle - 1.0);
    }

    /** Returns true once the homing sequence has completed successfully. */
    public boolean isArmHomed() {
        return m_HomingState == ArmHomedState.HOMED;
    }

    /** Stops the arm. */
    public void stop() {
        if (m_HomingState == ArmHomedState.HOMING) {
            DriverStation.reportWarning("ArmSubsystem: homing interrupted by stop() call — arm NOT homed. Press right stick to retry.", false);
            SmartDashboard.putString("Intake/ArmWarning", "Homing interrupted by stop() call — press right stick to retry");
            m_HomingState = ArmHomedState.NOT_HOMED;
            m_homingTimer.stop();
            m_armIO.setSoftLimitsEnabled(false);
        }
        m_armIO.stop();
    }


    /**
     * Runs one periodic tick of the homing state machine.
     * Creeps toward the hard stop at {@code kArmHomingSpeed}, watches for a sustained current
     * stall, then resets the encoder to 0 and transitions to HOMED.
     * Also enforces an overall timeout that transitions back to NOT_HOMED on failure.
     */
    private void homingHelper() {
        if (m_HomingState != ArmHomedState.HOMING) {
            return;
        }

        // Creep toward the hard stop (raised position). You may need to flip this sign once the mechanism is on the robot.
        m_armIO.moveArmWithSpeed(ArmConstants.kArmHomingSpeed);

        final double current = m_armOutputs.currentAmps;
        final double velocity = Math.abs(m_armOutputs.velocity);
        final boolean looksStalled = current >= ArmConstants.kArmHomingStallCurrent
                && velocity <= ArmConstants.kArmHomingStallVelocity;

        if (looksStalled) {
            if (m_stallStartSec < 0.0) {
                m_stallStartSec = m_homingTimer.get();
            }
        } else {
            m_stallStartSec = -1.0;
        }

        final boolean stallLongEnough = m_stallStartSec >= 0.0
                && (m_homingTimer.get() - m_stallStartSec) >= ArmConstants.kArmHomingStallSeconds;

        if (stallLongEnough) {
            // This stop is the mechanical reference (0). Adjust if your "zero" should be some other angle.
            m_armIO.resetEncoderValue();
            // Hold at stowed position with active position control instead of passive brake.
            m_armIO.setPosition(ArmConstants.kMinArmAngle);
            m_armIO.setSoftLimitsEnabled(true);
            m_HomingState = ArmHomedState.HOMED;
            m_homingTimer.stop();
            return;
        }

        if (m_homingTimer.get() >= ArmConstants.kArmHomingTimeoutSeconds) {
            // Timeout: stop and mark as NOT_HOMED so higher-level code can decide what to do.
            stop();
            m_HomingState = ArmHomedState.NOT_HOMED;
            m_homingTimer.stop();
            DriverStation.reportWarning("ArmSubsystem: homing timed out — arm NOT homed. Press right stick to retry.", false);
            SmartDashboard.putString("Intake/ArmWarning", "Homing timed out — press right stick to retry");
        }
    }
}
