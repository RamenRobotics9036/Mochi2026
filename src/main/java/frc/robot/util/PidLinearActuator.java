package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Linear actuator wrapper for the Actuonix L16-R series.
 *
 * <p>The L16-R is a self-contained RC servo actuator with internal position control.
 * It expects a standard RC PWM signal (1000–2000 µs pulse width) at 50 Hz.
 *
 * <p>This class uses {@link PWM#setSpeed(double)} as its core primitive, matching
 * the proven pattern from CD where {@code setSpeed(+1)} = fully extend (2000 µs)
 * and {@code setSpeed(-1)} = fully retract (1000 µs). The L16-R's internal
 * controller handles position tracking — you do not need to continuously send
 * commands for the actuator to reach and hold a position.
 *
 * <h3>Two usage patterns are supported:</h3>
 *
 * <h4>Pattern A — One-shot commands (simplest, matches CD proven pattern):</h4>
 * <pre>
 *   actuator.extendFully();   // call once, actuator moves on its own
 *   actuator.retractFully();  // call once
 *   actuator.setSpeedRaw(0.5); // call once, ~75% extended
 * </pre>
 *
 * <h4>Pattern B — Rate-limited mm-based API:</h4>
 * <pre>
 *   actuator.setTargetPositionMM(25.0);  // set target
 *   // then call actuator.update() every loop from periodic()
 * </pre>
 *
 * <p>Pattern B requires {@link #update()} to be called every robot loop.
 * Pattern A does not — just call the method once and the actuator handles it.
 *
 * <p>Bounds are configured as:
 * {@code setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000)} giving:
 * <ul>
 *   <li>{@code setSpeed(-1.0)} → 1000 µs (fully retracted)
 *   <li>{@code setSpeed(0.0)}  → 1500 µs (midpoint)
 *   <li>{@code setSpeed(+1.0)} → 2000 µs (fully extended)
 * </ul>
 */
public class PidLinearActuator {

    /** Maximum dt allowed in one update cycle (prevents first-loop large jump). */
    private static final double MAX_DT_SECONDS = 0.1;

    private final PWM m_pwm;
    private final String m_name;

    private final double m_minPositionMM;
    private final double m_maxPositionMM;
    private final double m_rangeMM;
    private final double m_maxRateMMPerSec;
    private final double m_positionToleranceMM;

    private double m_targetPositionMM;
    private double m_rateLimitedSetpointMM;
    private double m_lastSpeedCommand;
    private double m_lastUpdateTimestampSec;
    private boolean m_jogActive;
    private boolean m_skipNextJogIncrement;
    private boolean m_rateLimitingEnabled;

    /**
     * Constructs a PidLinearActuator backed by an L16-R servo actuator.
     *
     * @param name               Unique name used for SmartDashboard telemetry keys
     * @param pwmChannel         RoboRIO PWM port connected to the L16-R signal wire
     * @param minPositionMM      Software minimum position in mm (fully retracted end)
     * @param maxPositionMM      Software maximum position in mm (fully extended end)
     * @param maxRateMMPerSec    Maximum rate of setpoint change in mm/s (rate limiter,
     *                           only used when calling update() with the mm-based API)
     * @param positionToleranceMM Tolerance in mm for at-target detection
     */
    public PidLinearActuator(
            String name,
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

        m_name = name;
        m_pwm = new PWM(pwmChannel);

        // L16-R expects 1000 µs (retracted) to 2000 µs (extended) at 50 Hz.
        //
        // With setSpeed() as the primitive:
        //   setSpeed(+1.0) maps to max  = 2000 µs (fully extended)
        //   setSpeed( 0.0) maps to center = 1500 µs (midpoint)
        //   setSpeed(-1.0) maps to min  = 1000 µs (fully retracted)
        //
        // Deadband is collapsed to center (1500) since the L16-R is a position
        // actuator, not a continuous rotation device — no dead zone is needed.
        m_pwm.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);

        // k4X gives ~50 Hz period, which is the standard RC servo update rate
        // expected by the L16-R.
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);

        m_minPositionMM = minPositionMM;
        m_maxPositionMM = maxPositionMM;
        m_rangeMM = maxPositionMM - minPositionMM;
        m_maxRateMMPerSec = maxRateMMPerSec;
        m_positionToleranceMM = positionToleranceMM;

        m_targetPositionMM = minPositionMM;
        m_rateLimitedSetpointMM = minPositionMM;
        m_lastSpeedCommand = -1.0;
        m_lastUpdateTimestampSec = Timer.getFPGATimestamp();
        m_jogActive = false;
        m_skipNextJogIncrement = false;
        m_rateLimitingEnabled = false;

        // Command fully retracted on startup.
        commandSpeed(-1.0);
    }

    // ------------------------------------------------------------------
    // Pattern A: One-shot commands (no update() needed)
    // ------------------------------------------------------------------

    /**
     * Commands the actuator to fully extend (2000 µs).
     * The L16-R's internal controller will move to the endpoint and hold.
     * No need to call {@link #update()} — just call this once.
     */
    public void extendFully() {
        m_rateLimitingEnabled = false;
        m_targetPositionMM = m_maxPositionMM;
        m_rateLimitedSetpointMM = m_maxPositionMM;
        commandSpeed(1.0);
    }

    /**
     * Commands the actuator to fully retract (1000 µs).
     * The L16-R's internal controller will move to the endpoint and hold.
     * No need to call {@link #update()} — just call this once.
     */
    public void retractFully() {
        m_rateLimitingEnabled = false;
        m_targetPositionMM = m_minPositionMM;
        m_rateLimitedSetpointMM = m_minPositionMM;
        commandSpeed(-1.0);
    }

    /**
     * Commands the actuator to hold at the midpoint (1500 µs / ~50% stroke).
     * Call once — the actuator's internal controller holds this position.
     */
    public void hold() {
        m_rateLimitingEnabled = false;
        double midMM = (m_minPositionMM + m_maxPositionMM) / 2.0;
        m_targetPositionMM = midMM;
        m_rateLimitedSetpointMM = midMM;
        commandSpeed(0.0);
    }

    /**
     * Directly commands a speed value in [-1.0, +1.0] to the PWM output.
     * <ul>
     *   <li>{@code +1.0} = fully extend (2000 µs)
     *   <li>{@code  0.0} = midpoint (1500 µs)
     *   <li>{@code -1.0} = fully retract (1000 µs)
     * </ul>
     *
     * <p>Call once — the L16-R's internal controller moves to the position and holds.
     * Disables rate limiting; call {@link #setTargetPositionMM(double)} to re-enable it.
     *
     * @param speed value in [-1.0, +1.0]
     */
    public void setSpeedRaw(double speed) {
        m_rateLimitingEnabled = false;
        double clamped = MathUtil.clamp(speed, -1.0, 1.0);
        // Update mm tracking to match the raw command
        m_rateLimitedSetpointMM = speedToPositionMM(clamped);
        m_targetPositionMM = m_rateLimitedSetpointMM;
        commandSpeed(clamped);
    }

    // ------------------------------------------------------------------
    // Pattern B: Rate-limited mm-based API (requires update() every loop)
    // ------------------------------------------------------------------

    /**
     * Sets the absolute target position. Enables rate limiting — you must call
     * {@link #update()} every loop from {@code periodic()} for the setpoint to
     * advance and the actuator to move.
     *
     * @param positionMM desired position in mm
     * @return actual delta applied after clamping, in mm
     */
    public double setTargetPositionMM(double positionMM) {
        m_rateLimitingEnabled = true;
        m_jogActive = false;
        m_skipNextJogIncrement = false;
        return setTargetInternal(positionMM);
    }

    /**
     * Steps the target position by a fixed delta from the current target.
     * Enables rate limiting.
     *
     * @param deltaMM distance to step in mm (positive = extend, negative = retract)
     * @return actual delta applied after clamping, in mm
     */
    public double stepBy(double deltaMM) {
        m_rateLimitingEnabled = true;
        m_jogActive = false;
        m_skipNextJogIncrement = true;
        return setTargetInternal(m_targetPositionMM + deltaMM);
    }

    /**
     * Incrementally jogs the target position. Enables rate limiting.
     * Call repeatedly from a periodic command while a button is held.
     *
     * @param deltaMM per-cycle jog increment in mm
     * @return actual delta applied, {@code Double.NaN} if skipped, or {@code 0.0} if no delta
     */
    public double jogBy(double deltaMM) {
        m_rateLimitingEnabled = true;
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
     * Stops an active jog command.
     *
     * @return true if a jog was active and was stopped
     */
    public boolean stopJog() {
        m_skipNextJogIncrement = false;
        if (!m_jogActive) {
            return false;
        }
        m_jogActive = false;
        return true;
    }

    // ------------------------------------------------------------------
    // Periodic update — only needed for Pattern B (rate-limited mm API)
    // ------------------------------------------------------------------

    /**
     * Advances the rate-limited setpoint toward the target and writes the PWM command.
     *
     * <p><b>Only needed when using the mm-based API</b> ({@code setTargetPositionMM},
     * {@code stepBy}, {@code jogBy}). One-shot commands ({@code extendFully},
     * {@code retractFully}, {@code setSpeedRaw}) do not require this.
     *
     * <p>Call from subsystem {@code periodic()} every robot loop (~50 Hz).
     */
    public void update() {
        if (m_rateLimitingEnabled) {
            double currentTimeSec = Timer.getFPGATimestamp();
            double dtSeconds = MathUtil.clamp(
                currentTimeSec - m_lastUpdateTimestampSec, 0.0, MAX_DT_SECONDS);
            m_lastUpdateTimestampSec = currentTimeSec;

            advanceRateLimitedSetpoint(dtSeconds);

            double speed = positionMMToSpeed(m_rateLimitedSetpointMM);
            commandSpeed(speed);
        }

        // Always publish telemetry so dashboard works for both patterns.
        SmartDashboard.putNumber(m_name + "/TargetMM", m_targetPositionMM);
        SmartDashboard.putNumber(m_name + "/SetpointMM", m_rateLimitedSetpointMM);
        SmartDashboard.putNumber(m_name + "/SpeedCommand", m_lastSpeedCommand);
        SmartDashboard.putBoolean(m_name + "/AtTarget", isAtTarget());
        SmartDashboard.putBoolean(m_name + "/RateLimited", m_rateLimitingEnabled);
    }

    // ------------------------------------------------------------------
    // Getters
    // ------------------------------------------------------------------

    /** Returns the current target position in mm (after clamping). */
    public double getTargetPositionMM() {
        return m_targetPositionMM;
    }

    /**
     * Returns the current rate-limited setpoint in mm.
     * For one-shot commands, this equals the target.
     */
    public double getCurrentSetpointMM() {
        return m_rateLimitedSetpointMM;
    }

    /**
     * Returns the last raw speed command in [-1.0, +1.0] sent to the PWM output.
     */
    public double getRawSpeedCommand() {
        return m_lastSpeedCommand;
    }

    /**
     * Returns true when the rate-limited setpoint has reached the target within tolerance.
     * For one-shot commands, this is always true immediately after the command.
     */
    public boolean isAtTarget() {
        return Math.abs(m_targetPositionMM - m_rateLimitedSetpointMM) <= m_positionToleranceMM;
    }

    /** Disables the PWM output. The L16-R will hold its last position mechanically. */
    public void disable() {
        m_pwm.setDisabled();
    }

    // ------------------------------------------------------------------
    // Private helpers
    // ------------------------------------------------------------------

    /**
     * Sends a speed command to the PWM hardware.
     * This is the single point where all output goes through.
     *
     * @param speed value in [-1.0, +1.0]
     */
    private void commandSpeed(double speed) {
        m_lastSpeedCommand = MathUtil.clamp(speed, -1.0, 1.0);
        m_pwm.setSpeed(m_lastSpeedCommand);
    }

    /**
     * Converts mm position to setSpeed value [-1.0, +1.0].
     * minPositionMM → -1.0 (1000 µs), maxPositionMM → +1.0 (2000 µs).
     */
    private double positionMMToSpeed(double positionMM) {
        // Map [minMM, maxMM] → [-1.0, +1.0]
        double fraction = (positionMM - m_minPositionMM) / m_rangeMM; // 0..1
        return MathUtil.clamp(fraction * 2.0 - 1.0, -1.0, 1.0);
    }

    /**
     * Converts a setSpeed value [-1.0, +1.0] back to mm position.
     */
    private double speedToPositionMM(double speed) {
        // Map [-1.0, +1.0] → [minMM, maxMM]
        double fraction = (speed + 1.0) / 2.0; // 0..1
        return m_minPositionMM + fraction * m_rangeMM;
    }

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

    private double clampToLimits(double positionMM) {
        return MathUtil.clamp(positionMM, m_minPositionMM, m_maxPositionMM);
    }
}
