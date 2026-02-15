package frc.robot.visutils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.OptionalDouble;

/**
 * Stateful controller for aim-while-driving.
 *
 * <p>Encapsulates the heading PID and the "was aiming" edge-detection
 * flag so that:
 * <ul>
 *   <li>The PID resets exactly once on the rising edge of the aim button.</li>
 *   <li>When the target flickers (Limelight temporarily loses the tag),
 *       the PID is <b>not</b> reset — the derivative-term history
 *       survives, preventing jerk/oscillation.</li>
 *   <li>{@code m_wasAiming} is only cleared when the button is
 *       <b>released</b>.</li>
 * </ul>
 *
 * <p>Unit-testable independently of {@code RobotContainer}.
 */
public class AimController {
    private final PIDController m_pid;
    private final double m_maxOmega;
    private boolean m_wasAiming = false;

    /**
     * Creates an AimController with standard heading-PID gains.
     *
     * @param maxOmega maximum angular velocity to clamp to (rad/s)
     */
    public AimController(double maxOmega) {
        m_pid = TurnToAngleHelper.createAimPID();
        m_maxOmega = maxOmega;
    }

    /**
     * Call every cycle from the drive loop.
     *
     * <p>When the aim button is not pressed the internal state is
     * cleared and {@link OptionalDouble#empty()} is returned.
     * When the button <em>is</em> pressed but the target is lost,
     * the state is retained (PID not reset) and empty is returned.
     *
     * @param aimButtonPressed true when the driver is holding the aim button
     * @param tagId            last-seen AprilTag ID (−1 = none)
     * @param robotPose        current robot pose
     * @param robotSpeeds      robot-relative chassis speeds
     * @return the aim rate in rad/s, or empty if not aiming or no valid target
     */
    public OptionalDouble update(
            boolean aimButtonPressed, int tagId,
            Pose2d robotPose, ChassisSpeeds robotSpeeds) {

        if (!aimButtonPressed) {
            m_wasAiming = false;
            return OptionalDouble.empty();
        }

        // Rising edge: reset PID once when aim starts
        if (!m_wasAiming) {
            m_pid.reset();
            m_wasAiming = true;
        }

        // May return empty if the tag target is invalid/lost,
        // but we intentionally keep m_wasAiming = true so the
        // PID's derivative history survives target flicker.
        return TurnToAngleHelper.computeAimRateForTag(
            tagId, robotPose, robotSpeeds, m_pid, m_maxOmega);
    }
}
