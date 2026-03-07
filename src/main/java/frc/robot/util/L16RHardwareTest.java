package frc.robot.util;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Minimal hardware validation test for the Actuonix L16-R linear actuator.
 *
 * <p>Uses {@code PWM.setSpeed()} matching the CD-proven pattern.
 * Toggles between fully retracted and fully extended every 3 seconds.
 *
 * <p>Call {@link #initialize()} from {@code teleopInit()},
 * then {@link #periodic()} from {@code teleopPeriodic()}.
 * Remove once hardware is validated.
 */
public class L16RHardwareTest {

    // ===== CHANGE THIS to match your physical PWM port =====
    private static final int PWM_CHANNEL = 0;
    private static final double TOGGLE_PERIOD_SEC = 3.0;

    private final PWM m_pwm;
    private double m_startTimeSec;
    private boolean m_extended;

    public L16RHardwareTest() {
        m_pwm = new PWM(PWM_CHANNEL);
        // L16-R: 1000 µs retracted, 2000 µs extended
        m_pwm.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000);
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k4X); // 50 Hz
    }

    /** Call from {@code teleopInit()}. */
    public void initialize() {
        m_startTimeSec = Timer.getFPGATimestamp();
        m_extended = false;
        m_pwm.setSpeed(-1.0); // fully retract
        SmartDashboard.putString("L16R_Test/State", "RETRACTED (speed=-1 → 1000µs)");
    }

    /** Call from {@code teleopPeriodic()} every loop. */
    public void periodic() {
        double elapsed = Timer.getFPGATimestamp() - m_startTimeSec;
        boolean shouldExtend = ((int) (elapsed / TOGGLE_PERIOD_SEC) % 2) == 1;

        if (shouldExtend != m_extended) {
            m_extended = shouldExtend;
            if (m_extended) {
                m_pwm.setSpeed(1.0); // 2000 µs — fully extended
                SmartDashboard.putString("L16R_Test/State", "EXTENDED (speed=+1 → 2000µs)");
            } else {
                m_pwm.setSpeed(-1.0); // 1000 µs — fully retracted
                SmartDashboard.putString("L16R_Test/State", "RETRACTED (speed=-1 → 1000µs)");
            }
        }

        SmartDashboard.putNumber("L16R_Test/ElapsedSec", elapsed);
    }
}
