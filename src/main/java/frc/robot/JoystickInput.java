package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.sim.JoystickInputsRecord;
import frc.robot.sim.SimJoystickOrientation;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Processes raw joystick inputs into scaled robot velocities.
 *
 * <p>Pipeline per axis: raw supplier ? deadband with rescaling ? power response curve ?
 * slew-rate limit ? speed scale ? optional fine-positioning halving.
 *
 * <p>When running in simulation the outputs of {@link #getJoystickInputs()} are
 * additionally transformed via {@link SimJoystickOrientation} so that
 * "joystick-up" always drives toward the top of the Glass field view
 * regardless of alliance colour.
 *
 * <p>All external dependencies are injected through the constructor so this
 * class can be unit-tested without hardware or a running robot.
 */
public class JoystickInput {

    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRate);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationSlewRate);

    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;
    private final DoubleSupplier rawRotateSupplier;
    private final BooleanSupplier finePositioningEnabledSupplier;
    private final double teleoperatedSpeed;
    private final double maxAngularRate;
    private final boolean isSimulation;
    private final DoubleSupplier operatorForwardDegreesSupplier;

    /**
     * Creates a new JoystickInput processor.
     *
     * @param rawXSupplier             Supplier for the forward/backward axis (typically {@code -leftY}).
     * @param rawYSupplier             Supplier for the strafe axis (typically {@code -leftX}).
     * @param rawRotateSupplier        Supplier for the rotation axis (typically {@code -rightX}).
     * @param finePositioningEnabledSupplier  Returns {@code true} when fine-positioning mode is active
     *                                 (halves all output velocities).
     * @param teleoperatedSpeed        Maximum linear velocity in m/s.
     * @param maxAngularRate           Maximum angular velocity in rad/s.
     * @param isSimulation             {@code true} when running inside the WPILib simulator.
     * @param operatorForwardDegreesSupplier   Supplies the operator forward direction in degrees
     *                                          (used only when {@code isSimulation} is {@code true};
     *                                          may be {@code null} otherwise).
     */
    public JoystickInput(
            DoubleSupplier rawXSupplier,
            DoubleSupplier rawYSupplier,
            DoubleSupplier rawRotateSupplier,
            BooleanSupplier finePositioningEnabledSupplier,
            double teleoperatedSpeed,
            double maxAngularRate,
            boolean isSimulation,
            DoubleSupplier operatorForwardDegreesSupplier) {
        this.rawXSupplier = rawXSupplier;
        this.rawYSupplier = rawYSupplier;
        this.rawRotateSupplier = rawRotateSupplier;
        this.finePositioningEnabledSupplier = finePositioningEnabledSupplier;
        this.teleoperatedSpeed = teleoperatedSpeed;
        this.maxAngularRate = maxAngularRate;
        this.isSimulation = isSimulation;
        this.operatorForwardDegreesSupplier = operatorForwardDegreesSupplier;
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
     * Processes the forward/backward (X) axis.
     *
     * @return Scaled velocity in meters per second.
     */
    private double getDriveX() {
        double raw = rawXSupplier.getAsDouble();
        double deadbanded = applyDeadbandWithRescale(raw, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kTranslationExponent);
        double input = m_xLimiter.calculate(curved);
        double inputScale = finePositioningEnabledSupplier.getAsBoolean() ? 0.5 : 1.0;
        return input * teleoperatedSpeed * inputScale;
    }

    /**
     * Processes the strafe (Y) axis.
     *
     * @return Scaled velocity in meters per second.
     */
    private double getDriveY() {
        double raw = rawYSupplier.getAsDouble();
        double deadbanded = applyDeadbandWithRescale(raw, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kTranslationExponent);
        double input = m_yLimiter.calculate(curved);
        double inputScale = finePositioningEnabledSupplier.getAsBoolean() ? 0.5 : 1.0;
        return input * teleoperatedSpeed * inputScale;
    }

    /**
     * Processes the rotation axis.
     *
     * @return Scaled angular velocity in radians per second.
     */
    private double getDriveRotate() {
        double raw = rawRotateSupplier.getAsDouble();
        double deadbanded = applyDeadbandWithRescale(raw, DriveConstants.kJoystickDeadband);
        double curved = applyResponseCurve(deadbanded, DriveConstants.kRotationExponent);
        double input = m_rotLimiter.calculate(curved);
        double inputScale = finePositioningEnabledSupplier.getAsBoolean() ? 0.5 : 1.0;
        return input * maxAngularRate * inputScale;
    }

    /**
     * Returns all three processed axes as a single record.
     *
     * <p>In simulation mode the translation axes are additionally transformed
     * by {@link SimJoystickOrientation} so that "joystick-up" maps to
     * "screen-up" in the Glass field view.
     *
     * @return A {@link JoystickInputsRecord} with driveX, driveY, and rotateX.
     */
    public JoystickInputsRecord getJoystickInputs() {
        double x = getDriveX();
        double y = getDriveY();
        double rot = getDriveRotate();

        if (isSimulation) {
            JoystickInputsRecord transformed =
                SimJoystickOrientation.simTransformJoystickOrientation(
                    operatorForwardDegreesSupplier.getAsDouble(), x, y, rot);
            x   = -1 * transformed.driveX();
            y   = -1 * transformed.driveY();
            rot = transformed.rotatetX();
        }

        return new JoystickInputsRecord(x, y, rot);
    }

    /** Returns {@code true} when {@code controller}'s D-pad is in the downward region (135°–225°). */
    public static boolean isPovDownward(CommandXboxController controller) {
        int pov = controller.getHID().getPOV();
        return pov != -1 && (pov >= 135 && pov <= 225);
    }

    /** Returns {@code true} when {@code controller}'s D-pad is in the upward region (315°–360° or 0°–45°). */
    public static boolean isPovUpward(CommandXboxController controller) {
        int pov = controller.getHID().getPOV();
        return pov != -1 && (pov <= 45 || pov >= 315);
    }
}
