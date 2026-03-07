package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

/**
 * Linear actuator wrapper for the Actuonix L16-R series.
 *
 * <p>The L16-R is a self-contained RC servo actuator with internal position control.
 * It expects a standard RC PWM signal (1.0ms–2.0ms pulse width), identical to a hobby servo.
 * No external PID, potentiometer, or motor controller is required or appropriate.
 *
 * <p>WPILib's {@link Servo} class produces the correct PWM signal.
 * {@code set(0.0)} = fully retracted, {@code set(1.0)} = fully extended.
 *
 * <p>This class provides:
 * <ul>
 *   <li>Software position limits in mm
 *   <li>Rate-limited setpoint advancement (to avoid slamming the actuator)
 *   <li>Jog and discrete step movement API
 *   <li>At-target detection based on setpoint proximity
 * </ul>
 */
public class PidLinearActuator {

    private final Servo m_actuator;

    private final double m_minPositionMM;
    private final double m_maxPositionMM;
    private final double m_rangeMM;
    private final double m_maxRateMMPerSec;
    private final double m_positionToleranceMM;

    private double m_targetPositionMM;
    private double m_rateLimitedSetpointMM;
    private double m_lastUpdateTimestampSec;
    private boolean m_jogActive;
    private boolean m_skipNextJogIncrement;

    /**
     * Constructs a PidLinearActuator backed by an L16-R servo actuator.
     *
     * @param pwmChannel         RoboRIO PWM port connected to the L16-R signal wire
     * @param minPositionMM      Software minimum position in mm (fully retracted end)
     * @param maxPositionMM      Software maximum position in mm (fully extended end)
     * @param maxRateMMPerSec    Maximum rate of setpoint change in mm/s (rate limiter)
     * @param positionToleranceMM Tolerance in mm for at-target detection
     */
    public PidLinearActuator(
            int pwmChannel,
            double minPositionMM,
            double maxPositionMM,
            double maxRateMMPerSec,
            double positionToleranceMM) {

        if (maxPositionMM <= minPositionMM) {
            throw new IllegalArgumentException("maxPositionMM must be > minPositionMM.");
        }
        if (maxRateMMPerSec <= 0.0) {
            throw new IllegalArgumentException("maxRateMMPerSec must be positive.");
        }
        if (positionToleranceMM < 0.0) {
            throw new IllegalArgumentException("positionToleranceMM cannot be negative.");
        }

        m_actuator = new Servo(pwmChannel);

        // L16-R requires standard RC PWM: 1000µs (retracted) to 2000µs (extended).
        // WPILib's Servo default range is ~600µs–2400µs, which the L16-R rejects as
        // out-of-bounds and disables its motor entirely. This must be set explicitly.
        m_actuator.setBoundsMicroseconds(2000, 1520, 1500, 1480, 1000);

        m_minPositionMM = minPositionMM;
        m_maxPositionMM = maxPositionMM;
        m_rangeMM = maxPositionMM - minPositionMM;
        m_maxRateMMPerSec = maxRateMMPerSec;
        m_positionToleranceMM = positionToleranceMM;

        // Start at the retracted (minimum) position
        m_targetPositionMM = minPositionMM;
        m_rateLimitedSetpointMM = minPositionMM;
        m_lastUpdateTimestampSec = Timer.getFPGATimestamp();
        m_jogActive = false;
        m_skipNextJogIncrement = false;

        // Command servo to retracted position on startup
        m_actuator.set(positionToServoFraction(minPositionMM));
    }

    /**
     * Steps the target position by a fixed delta from the current target.
     * Suppresses the next jog increment to avoid conflict with jog commands.
     *
     * @param deltaMM distance to step in mm (positive = extend, negative = retract)
     * @return actual delta applied after clamping, in mm
     */
    public double stepBy(double deltaMM) {
        m_jogActive = false;
        m_skipNextJogIncrement = true;
        return setTargetInternal(m_targetPositionMM + deltaMM);
    }

    /**
     * Sets the absolute target position directly.
     *
     * @param positionMM desired position in mm
     * @return actual delta applied after clamping, in mm
     */
    public double setTargetPositionMM(double positionMM) {
        m_jogActive = false;
        m_skipNextJogIncrement = false;
        return setTargetInternal(positionMM);
    }

    /**
     * Incrementally jogs the target position. Called repeatedly from a periodic command
     * while a button is held.
     *
     * @param deltaMM per-cycle jog increment in mm
     * @return actual delta applied, {@code Double.NaN} if skipped, or {@code 0.0} if no delta
     */
    public double jogBy(double deltaMM) {
        if (deltaMM == 0.0) {
            return 0.0;
        }
        if (m_skipNextJogIncrement) {
            m_skipNextJogIncrement = false;
            return Double.NaN;
        }
        m_jogActive = true;
        return setTargetInternal(m_targetPositionMM + deltaMM);
    }

    /**
     * Stops an active jog and holds the current rate-limited setpoint.
     *
     * @return true if a jog was active and was stopped
     */
    public boolean stopJog() {
        m_skipNextJogIncrement = false;
        if (!m_jogActive) {
            return false;
        }
        m_jogActive = false;
        // Hold wherever the rate-limited setpoint currently is
        setTargetInternal(m_rateLimitedSetpointMM);
        return true;
    }

    /**
     * Must be called every robot loop (e.g. from subsystem {@code periodic()}).
     * Advances the rate-limited setpoint and commands the servo.
     */
    public void update() {
        double currentTimeSec = Timer.getFPGATimestamp();
        double dtSeconds = Math.max(0.0, currentTimeSec - m_lastUpdateTimestampSec);

        advanceRateLimitedSetpoint(dtSeconds);

        double servoFraction = positionToServoFraction(m_rateLimitedSetpointMM);
        m_actuator.set(servoFraction);

        System.out.printf(
            "[L16-R] target=%.2fmm | setpoint=%.2fmm | servo=%.3f | atTarget=%b%n",
            m_targetPositionMM, m_rateLimitedSetpointMM, servoFraction, isAtTarget());

        m_lastUpdateTimestampSec = currentTimeSec;
    }

    /** Returns the current target position in mm (after clamping). */
    public double getTargetPositionMM() {
        return m_targetPositionMM;
    }

    /**
     * Returns the current rate-limited setpoint in mm.
     * This is what is actually being commanded to the servo right now.
     */
    public double getCurrentSetpointMM() {
        return m_rateLimitedSetpointMM;
    }

    /**
     * Returns true when the rate-limited setpoint has reached the target within tolerance.
     * Because the L16-R has internal position control, this is a reasonable proxy for
     * physical at-target — the servo will be tracking the setpoint internally.
     */
    public boolean isAtTarget() {
        return Math.abs(m_targetPositionMM - m_rateLimitedSetpointMM) <= m_positionToleranceMM;
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    private double setTargetInternal(double requestedPositionMM) {
        double previousTargetMM = m_targetPositionMM;
        m_targetPositionMM = clampToLimits(requestedPositionMM);
        return m_targetPositionMM - previousTargetMM;
    }

    private void advanceRateLimitedSetpoint(double dtSeconds) {
        if (dtSeconds <= 0.0) {
            return;
        }
        double maxStepMM = m_maxRateMMPerSec * dtSeconds;
        double error = m_targetPositionMM - m_rateLimitedSetpointMM;
        if (Math.abs(error) <= maxStepMM) {
            m_rateLimitedSetpointMM = m_targetPositionMM;
        } else {
            m_rateLimitedSetpointMM += Math.copySign(maxStepMM, error);
        }
    }

    /**
     * Converts a position in mm to a servo fraction [0.0, 1.0].
     * 0.0 = fully retracted (minPositionMM), 1.0 = fully extended (maxPositionMM).
     */
    private double positionToServoFraction(double positionMM) {
        return MathUtil.clamp((positionMM - m_minPositionMM) / m_rangeMM, 0.0, 1.0);
    }

    private double clampToLimits(double positionMM) {
        return MathUtil.clamp(positionMM, m_minPositionMM, m_maxPositionMM);
    }
}
