package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.PidLinearActuator;

/** Controls the shooter hood actuator with analog-feedback PID. */
public class HoodSubsystem extends SubsystemBase {
    private static final double MOTION_EPSILON_MM = 1e-6;

    private enum MotionAction {
        HOLD,
        STEP,
        JOG,
        DIRECT
    }

    private final PidLinearActuator m_hoodActuator = new PidLinearActuator(
        HoodConstants.HOOD_PWM_CHANNEL,
        HoodConstants.HOOD_POT_CHANNEL,
        HoodConstants.HOOD_MOTOR_INVERTED,
        HoodConstants.HOOD_POT_RANGE_MM,
        HoodConstants.HOOD_POT_OFFSET_MM,
        HoodConstants.HOOD_MIN_MM,
        HoodConstants.HOOD_MAX_MM,
        HoodConstants.HOOD_MAX_RATE_MM_PER_SEC,
        HoodConstants.HOOD_LOOP_PERIOD_SEC,
        HoodConstants.HOOD_kP,
        HoodConstants.HOOD_kI,
        HoodConstants.HOOD_kD,
        HoodConstants.HOOD_POSITION_TOLERANCE_MM,
        HoodConstants.HOOD_VELOCITY_TOLERANCE_MM_PER_SEC,
        HoodConstants.HOOD_MAX_PID_OUTPUT,
        HoodConstants.HOOD_MIN_MOVEMENT_MM,
        HoodConstants.HOOD_STALL_TIMEOUT_SEC,
        HoodConstants.HOOD_STALL_OUTPUT_THRESHOLD);

    private MotionAction m_motionAction = MotionAction.HOLD;
    private boolean m_wasAtTarget;

    public HoodSubsystem() {
        m_wasAtTarget = m_hoodActuator.isAtTarget();
    }

    public void stepHoodExtend() {
        double requestedDeltaMM = HoodConstants.HOOD_STEP_MM;
        double appliedDeltaMM = m_hoodActuator.stepBy(requestedDeltaMM);
        logMotionCommand(MotionAction.STEP, "step-extend", requestedDeltaMM, appliedDeltaMM);
    }

    public void stepHoodRetract() {
        double requestedDeltaMM = -HoodConstants.HOOD_STEP_MM;
        double appliedDeltaMM = m_hoodActuator.stepBy(requestedDeltaMM);
        logMotionCommand(MotionAction.STEP, "step-retract", requestedDeltaMM, appliedDeltaMM);
    }

    public void jogExtend() {
        double requestedDeltaMM = HoodConstants.HOOD_JOG_MM_PER_TICK;
        double appliedDeltaMM = m_hoodActuator.jogBy(requestedDeltaMM);
        logMotionCommand(MotionAction.JOG, "jog-extend", requestedDeltaMM, appliedDeltaMM);
    }

    public void jogRetract() {
        double requestedDeltaMM = -HoodConstants.HOOD_JOG_MM_PER_TICK;
        double appliedDeltaMM = m_hoodActuator.jogBy(requestedDeltaMM);
        logMotionCommand(MotionAction.JOG, "jog-retract", requestedDeltaMM, appliedDeltaMM);
    }

    public void stopJog() {
        if (!m_hoodActuator.stopJog()) {
            return;
        }

        m_motionAction = MotionAction.HOLD;
        m_wasAtTarget = true;
        DataLogManager.log(String.format(
            "Hood jog-stop | time=%.3f s | hold=%.2f mm | baseline=%.2f mm",
            Timer.getFPGATimestamp(),
            getHoodPosition(),
            getHoodBaselinePosition()));
    }

    public void setHoodPosition(double positionMM) {
        double requestedDeltaMM = positionMM - getHoodTargetPosition();
        double appliedDeltaMM = m_hoodActuator.setTargetPositionMM(positionMM);
        logMotionCommand(MotionAction.DIRECT, "direct-set", requestedDeltaMM, appliedDeltaMM);
    }

    public double getHoodPosition() {
        return m_hoodActuator.getCurrentPositionMM();
    }

    public double getHoodTargetPosition() {
        return m_hoodActuator.getTargetPositionMM();
    }

    public double getHoodBaselinePosition() {
        return m_hoodActuator.getBaselinePositionMM();
    }

    public boolean isHoodAtTarget() {
        return m_hoodActuator.isAtTarget();
    }

    @Override
    public void periodic() {
        m_hoodActuator.update();

        if (m_hoodActuator.consumeStallEvent()) {
            DataLogManager.log(String.format(
                "Hood stall-hold | time=%.3f s | pos=%.2f mm | baseline=%.2f mm",
                Timer.getFPGATimestamp(),
                getHoodPosition(),
                getHoodBaselinePosition()));
            m_motionAction = MotionAction.HOLD;
            m_wasAtTarget = true;
            return;
        }

        boolean atTarget = isHoodAtTarget();
        if (!m_wasAtTarget && atTarget) {
            DataLogManager.log(String.format(
                "Hood settled | time=%.3f s | action=%s | pos=%.2f mm | target=%.2f mm | baseline=%.2f mm",
                Timer.getFPGATimestamp(),
                m_motionAction.name(),
                getHoodPosition(),
                getHoodTargetPosition(),
                getHoodBaselinePosition()));
            m_motionAction = MotionAction.HOLD;
        }

        m_wasAtTarget = atTarget;
    }

    private void logMotionCommand(
            MotionAction action,
            String label,
            double requestedDeltaMM,
            double appliedDeltaMM) {
        if (Double.isNaN(appliedDeltaMM)) {
            return;
        }
        if (Math.abs(requestedDeltaMM) <= MOTION_EPSILON_MM
            && Math.abs(appliedDeltaMM) <= MOTION_EPSILON_MM) {
            return;
        }

        m_motionAction = action;
        DataLogManager.log(String.format(
            "Hood %s | time=%.3f s | requestedDelta=%.2f mm | appliedDelta=%.2f mm | pos=%.2f mm | target=%.2f mm | baseline=%.2f mm",
            label,
            Timer.getFPGATimestamp(),
            requestedDeltaMM,
            appliedDeltaMM,
            getHoodPosition(),
            getHoodTargetPosition(),
            getHoodBaselinePosition()));
    }
}
