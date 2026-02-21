package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.sim.armsim.ArmIoInterface;

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
        if(m_HomingState == ArmHomedState.HOMING) {
            homingHelper();
        }
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

    /** Homes the arm by moving it to the zero position. */
    public void beginHoming() {
        if(m_HomingState != ArmHomedState.HOMING) {
            m_HomingState = ArmHomedState.HOMING;
            m_homingTimer.restart();
            m_stallStartSec = -1.0;
        }
    }

    /** Sets the desired position for the arm. */
    public void setArmPosition(double position) {
        m_armIO.setPosition(
            MathUtil.clamp(position, ArmConstants.kMinArmAngle, ArmConstants.kMaxArmAngle));
    }

    /** Checks if the arm is deployed. */
    public boolean isArmDeployed() {
        return m_armOutputs.position >= (ArmConstants.kMaxArmAngle - 1.0);
    }

    /** Checks if arm is homed. */
    public boolean isArmHomed() {
        return m_armOutputs.position <= (ArmConstants.kMinArmAngle + 1.0);
    }

    /** Stops the arm. */
    public void stop(){
        m_armIO.stop();
    }


    //TODO: get whoever wrote this to add doc comment.
    public void homingHelper() {
        if (m_HomingState != ArmHomedState.HOMING) {
            return;
        }

        // Creep toward the hard stop. You may need to flip this sign once the mechanism is on the robot.
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
            stop();
            // This stop is the mechanical reference (0). Adjust if your "zero" should be some other angle.
            m_armIO.resetEncoderValue();
            m_HomingState = ArmHomedState.HOMED;
            m_homingTimer.stop();
            return;
        }

        if (m_homingTimer.get() >= ArmConstants.kArmHomingTimeoutSeconds) {
            // Timeout: stop and mark as NOT_HOMED so higher-level code can decide what to do.
            stop();
            m_HomingState = ArmHomedState.NOT_HOMED;
            m_homingTimer.stop();
        }
    }
}
