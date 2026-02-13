package frc.robot.sim;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.DriveConstants;

/**
 * Utility class for processing joystick inputs with smooth driving curves.
 * Applies deadband with rescaling, power-based response curves, and slew rate limiting.
 */
public class DriveSmooth {
    private final SlewRateLimiter m_xLimiter;
    private final SlewRateLimiter m_yLimiter;
    private final SlewRateLimiter m_rotLimiter;

    public DriveSmooth() {
        m_xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        m_yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
        m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationSlewRate);
    }

    /**
     * Applies deadband with linear rescaling to preserve full output range.
     * Maps [deadband, 1.0] to [0.0, 1.0] so no top-end speed is lost.
     *
     * @param value Raw joystick input (-1 to 1)
     * @param deadband Deadband threshold (e.g., 0.1 for 10%)
     * @return Processed value with deadband applied and rescaled to full range
     */
    private double applyDeadbandWithRescale(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return Math.signum(value) * ((Math.abs(value) - deadband) / (1.0 - deadband));
    }

    /**
     * Applies power-based response curve for fine control at low speeds.
     *
     * @param value Processed input (-1 to 1)
     * @param exponent Response curve exponent (1.0=linear, 2.0=squared, 3.0=cubed)
     * @return Curved value
     */
    private double applyResponseCurve(double value, double exponent) {
        return Math.signum(value) * Math.pow(Math.abs(value), exponent);
    }

    /**
     * Processes translation X input (forward/backward).
     *
     * @param rawInput Raw joystick value (-1 to 1)
     * @return Smoothed value (-1 to 1)
     */
    public double processTranslationX(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kTranslationExponent);
        return m_xLimiter.calculate(curved);
    }

    /**
     * Processes translation Y input (left/right strafe).
     *
     * @param rawInput Raw joystick value (-1 to 1)
     * @return Smoothed value (-1 to 1)
     */
    public double processTranslationY(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kTranslationExponent);
        return m_yLimiter.calculate(curved);
    }

    /**
     * Processes rotation input.
     *
     * @param rawInput Raw joystick value (-1 to 1)
     * @return Smoothed value (-1 to 1)
     */
    public double processRotation(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kRotationExponent);
        return m_rotLimiter.calculate(curved);
    }

    /**
     * Resets all slew rate limiters to zero.
     * Call when transitioning from autonomous to teleop or when the robot is re-enabled.
     */
    public void reset() {
        m_xLimiter.reset(0);
        m_yLimiter.reset(0);
        m_rotLimiter.reset(0);
    }
}
