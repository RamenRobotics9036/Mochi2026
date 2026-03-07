package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.PidLinearActuator;

/** Controls the shooter hood using an Actuonix L16-R servo actuator. */
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
        HoodConstants.HOOD_MIN_MM,
        HoodConstants.HOOD_MAX_MM,
        HoodConstants.HOOD_MAX_RATE_MM_PER_SEC,
        HoodConstants.HOOD_POSITION_TOLERANCE_MM);

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
            "Hood jog-stop | time=%.3f s | setpoint=%.2f mm",
            Timer.getFPGATimestamp(),
            m_hoodActuator.getCurrentSetpointMM()));
    }

    public void setHoodPosition(double positionMM) {
        double requestedDeltaMM = positionMM - getHoodTargetPosition();
        double appliedDeltaMM = m_hoodActuator.setTargetPositionMM(positionMM);
        logMotionCommand(MotionAction.DIRECT, "direct-set", requestedDeltaMM, appliedDeltaMM);
    }

    /** Returns the current rate-limited setpoint (what is being sent to the servo now). */
    public double getHoodPosition() {
        return m_hoodActuator.getCurrentSetpointMM();
    }

    public double getHoodTargetPosition() {
        return m_hoodActuator.getTargetPositionMM();
    }

    public boolean isHoodAtTarget() {
        return m_hoodActuator.isAtTarget();
    }

    @Override
    public void periodic() {
        m_hoodActuator.update();

        boolean atTarget = isHoodAtTarget();
        if (!m_wasAtTarget && atTarget) {
            DataLogManager.log(String.format(
                "Hood settled | time=%.3f s | action=%s | setpoint=%.2f mm | target=%.2f mm",
                Timer.getFPGATimestamp(),
                m_motionAction.name(),
                m_hoodActuator.getCurrentSetpointMM(),
                getHoodTargetPosition()));
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
            "Hood %s | time=%.3f s | requestedDelta=%.2f mm | appliedDelta=%.2f mm | setpoint=%.2f mm | target=%.2f mm",
            label,
            Timer.getFPGATimestamp(),
            requestedDeltaMM,
            appliedDeltaMM,
            m_hoodActuator.getCurrentSetpointMM(),
            getHoodTargetPosition()));
    }
}
