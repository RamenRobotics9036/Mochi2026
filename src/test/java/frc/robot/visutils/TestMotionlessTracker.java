package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.VisionKalmanConstants;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.*;

/**
 * Unit tests for {@link MotionlessTracker}.
 *
 * <p>Thresholds (from {@link VisionKalmanConstants}):
 * <ul>
 *   <li>Linear: 0.05 m/s</li>
 *   <li>Angular: 2.0 deg/s (≈ 0.0349 rad/s)</li>
 * </ul>
 *
 * <p>FPGA time is controlled via {@link SimHooks#pauseTiming()} /
 * {@link SimHooks#stepTiming(double)} so {@code Timer.getFPGATimestamp()}
 * advances deterministically.
 */
class TestMotionlessTracker {

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** Mutable speeds that the tracker reads each cycle. */
    private ChassisSpeeds m_speeds;
    private MotionlessTracker m_tracker;

    @BeforeEach
    void setUp() {
        m_speeds = new ChassisSpeeds();          // all zeros → motionless
        m_tracker = new MotionlessTracker(() -> m_speeds);
    }

    // -- helpers --

    /** Sets linear speed only (angular = 0). */
    private void setLinearSpeed(double vx, double vy) {
        m_speeds = new ChassisSpeeds(vx, vy, 0.0);
    }

    /** Sets angular speed only (linear = 0). */
    private void setAngularSpeedDeg(double degPerSec) {
        m_speeds = new ChassisSpeeds(0, 0, Math.toRadians(degPerSec));
    }

    /** Sets both linear and angular speeds. */
    private void setSpeeds(double vx, double vy, double omegaRad) {
        m_speeds = new ChassisSpeeds(vx, vy, omegaRad);
    }

    // -----------------------------------------------------------------------
    // 1. Basic motionless detection
    // -----------------------------------------------------------------------

    @Test
    void zeroSpeeds_isMotionless() {
        m_tracker.update();
        assertTrue(m_tracker.isMotionless());
    }

    @Test
    void belowBothThresholds_isMotionless() {
        setLinearSpeed(0.01, 0.01);                     // hypot ≈ 0.014 < 0.05
        setAngularSpeedDeg(1.0);                        // 1 < 2 deg/s
        // combine into one ChassisSpeeds
        m_speeds = new ChassisSpeeds(0.01, 0.01, Math.toRadians(1.0));
        m_tracker.update();
        assertTrue(m_tracker.isMotionless());
    }

    @Test
    void linearAboveThreshold_isNotMotionless() {
        setLinearSpeed(0.1, 0.0);                       // 0.1 > 0.05
        m_tracker.update();
        assertFalse(m_tracker.isMotionless());
    }

    @Test
    void angularAboveThreshold_isNotMotionless() {
        setAngularSpeedDeg(3.0);                        // 3 > 2
        m_tracker.update();
        assertFalse(m_tracker.isMotionless());
    }

    @Test
    void bothAboveThresholds_isNotMotionless() {
        setSpeeds(1.0, 1.0, Math.toRadians(10.0));
        m_tracker.update();
        assertFalse(m_tracker.isMotionless());
    }

    // -----------------------------------------------------------------------
    // 2. Threshold boundary behavior
    // -----------------------------------------------------------------------

    @Test
    void linearExactlyAtThreshold_isMotionless() {
        // Linear speed == threshold (strict < comparison means this is NOT motionless)
        double threshold = VisionKalmanConstants.kMotionlessLinearThreshold;
        setLinearSpeed(threshold, 0.0);
        m_tracker.update();
        // 0.05 is NOT < 0.05, so should be moving
        assertFalse(m_tracker.isMotionless(),
            "Linear speed exactly at threshold should NOT count as motionless (strict <)");
    }

    @Test
    void linearJustBelowThreshold_isMotionless() {
        double threshold = VisionKalmanConstants.kMotionlessLinearThreshold;
        setLinearSpeed(threshold - 0.001, 0.0);
        m_tracker.update();
        assertTrue(m_tracker.isMotionless());
    }

    @Test
    void angularExactlyAtThreshold_isNotMotionless() {
        double thresholdDeg = VisionKalmanConstants.kMotionlessGyroThreshold;
        setAngularSpeedDeg(thresholdDeg);
        m_tracker.update();
        assertFalse(m_tracker.isMotionless(),
            "Angular speed exactly at threshold should NOT count as motionless (strict <)");
    }

    @Test
    void angularJustBelowThreshold_isMotionless() {
        double thresholdDeg = VisionKalmanConstants.kMotionlessGyroThreshold;
        setAngularSpeedDeg(thresholdDeg - 0.01);
        m_tracker.update();
        assertTrue(m_tracker.isMotionless());
    }

    @Test
    void negativeAngularSpeed_usesAbsoluteValue() {
        // Negative angular velocity should be treated the same as positive.
        setAngularSpeedDeg(-3.0);
        m_tracker.update();
        assertFalse(m_tracker.isMotionless(),
            "Negative angular speed exceeding threshold should count as moving");
    }

    @Test
    void negativeLinearSpeeds_usesHypot() {
        // Negative vx/vy — hypot is always positive.
        setLinearSpeed(-0.04, -0.04);    // hypot ≈ 0.057 > 0.05
        m_tracker.update();
        assertFalse(m_tracker.isMotionless());
    }

    // -----------------------------------------------------------------------
    // 3. Edge detection & callback
    // -----------------------------------------------------------------------

    @Test
    void stillToMoving_firesCallback() {
        AtomicInteger callCount = new AtomicInteger(0);
        m_tracker.setOnStartedMoving(callCount::incrementAndGet);

        // Cycle 1: still
        m_tracker.update();
        assertTrue(m_tracker.isMotionless());
        assertEquals(0, callCount.get(), "No callback on first still cycle");

        // Cycle 2: start moving
        setLinearSpeed(1.0, 0.0);
        m_tracker.update();
        assertFalse(m_tracker.isMotionless());
        assertEquals(1, callCount.get(), "Callback should fire on still→moving transition");
    }

    @Test
    void movingToStill_doesNotFireCallback() {
        AtomicInteger callCount = new AtomicInteger(0);
        m_tracker.setOnStartedMoving(callCount::incrementAndGet);

        // Start moving
        setLinearSpeed(1.0, 0.0);
        m_tracker.update();

        // Become still
        setLinearSpeed(0.0, 0.0);
        m_tracker.update();
        assertEquals(0, callCount.get(), "Callback should NOT fire on moving→still");
    }

    @Test
    void stayingStill_doesNotFireCallback() {
        AtomicInteger callCount = new AtomicInteger(0);
        m_tracker.setOnStartedMoving(callCount::incrementAndGet);

        // Multiple still cycles
        for (int i = 0; i < 5; i++) {
            m_tracker.update();
        }
        assertEquals(0, callCount.get(), "No callback while continuously still");
    }

    @Test
    void stayingMoving_doesNotFireCallback() {
        AtomicInteger callCount = new AtomicInteger(0);
        m_tracker.setOnStartedMoving(callCount::incrementAndGet);

        setLinearSpeed(1.0, 0.0);
        for (int i = 0; i < 5; i++) {
            m_tracker.update();
        }
        assertEquals(0, callCount.get(), "No callback while continuously moving");
    }

    @Test
    void multipleTransitions_firesCallbackEachTime() {
        AtomicInteger callCount = new AtomicInteger(0);
        m_tracker.setOnStartedMoving(callCount::incrementAndGet);

        // still → moving (fires)
        m_tracker.update();
        setLinearSpeed(1.0, 0.0);
        m_tracker.update();
        assertEquals(1, callCount.get());

        // moving → still (no fire)
        setLinearSpeed(0.0, 0.0);
        m_tracker.update();
        assertEquals(1, callCount.get());

        // still → moving again (fires again)
        setLinearSpeed(1.0, 0.0);
        m_tracker.update();
        assertEquals(2, callCount.get());
    }

    @Test
    void noCallbackSet_stillToMovingDoesNotThrow() {
        // No callback registered — should not NPE on still→moving.
        m_tracker.update();               // still
        setLinearSpeed(1.0, 0.0);
        assertDoesNotThrow(() -> m_tracker.update());
    }

    @Test
    void callbackCanBeCleared() {
        AtomicInteger callCount = new AtomicInteger(0);
        m_tracker.setOnStartedMoving(callCount::incrementAndGet);

        // First transition fires
        m_tracker.update();
        setLinearSpeed(1.0, 0.0);
        m_tracker.update();
        assertEquals(1, callCount.get());

        // Clear callback
        m_tracker.setOnStartedMoving(null);

        // Second transition should NOT fire
        setLinearSpeed(0.0, 0.0);
        m_tracker.update();
        setLinearSpeed(1.0, 0.0);
        m_tracker.update();
        assertEquals(1, callCount.get(), "Callback should not fire after being cleared");
    }

    // -----------------------------------------------------------------------
    // 4. Duration tracking (getSecondsStill)
    // -----------------------------------------------------------------------

    @Test
    void getSecondsStill_returnsZeroWhenMoving() {
        setLinearSpeed(1.0, 0.0);
        m_tracker.update();
        assertEquals(0.0, m_tracker.getSecondsStill(), 1e-6);
    }

    @Test
    void getSecondsStill_increasesOverTime() {
        SimHooks.pauseTiming();
        try {
            // Become still
            m_tracker.update();
            assertTrue(m_tracker.isMotionless());

            // Advance 0.5 s
            SimHooks.stepTiming(0.5);
            double duration = m_tracker.getSecondsStill();
            assertEquals(0.5, duration, 0.05,
                "Should report ~0.5 s of stillness");

            // Advance another 1.0 s
            SimHooks.stepTiming(1.0);
            duration = m_tracker.getSecondsStill();
            assertEquals(1.5, duration, 0.05,
                "Should report ~1.5 s of stillness");
        } finally {
            SimHooks.resumeTiming();
        }
    }

    @Test
    void getSecondsStill_resetsWhenMovementStarts() {
        SimHooks.pauseTiming();
        try {
            // Still for 1 second
            m_tracker.update();
            SimHooks.stepTiming(1.0);
            assertTrue(m_tracker.getSecondsStill() > 0.5);

            // Start moving
            setLinearSpeed(1.0, 0.0);
            m_tracker.update();
            assertEquals(0.0, m_tracker.getSecondsStill(), 1e-6,
                "Duration should reset to 0 when moving");
        } finally {
            SimHooks.resumeTiming();
        }
    }

    @Test
    void getSecondsStill_resetsOnNewStillPeriod() {
        SimHooks.pauseTiming();
        try {
            // First still period
            m_tracker.update();
            SimHooks.stepTiming(2.0);
            double first = m_tracker.getSecondsStill();
            assertTrue(first > 1.5, "Should have ~2 s of stillness");

            // Move briefly
            setLinearSpeed(1.0, 0.0);
            m_tracker.update();
            SimHooks.stepTiming(0.1);

            // Become still again — timer should restart
            setLinearSpeed(0.0, 0.0);
            m_tracker.update();
            SimHooks.stepTiming(0.3);

            double second = m_tracker.getSecondsStill();
            assertTrue(second < 0.5,
                "New still period should start fresh, got " + second);
        } finally {
            SimHooks.resumeTiming();
        }
    }

    // -----------------------------------------------------------------------
    // 5. Initial state
    // -----------------------------------------------------------------------

    @Test
    void beforeFirstUpdate_isNotMotionless() {
        // isMotionless should return false before update() is ever called,
        // because m_isCurrentlyStill defaults to false.
        assertFalse(m_tracker.isMotionless(),
            "Should not be motionless before first update()");
    }

    @Test
    void beforeFirstUpdate_getSecondsStillReturnsZero() {
        assertEquals(0.0, m_tracker.getSecondsStill(), 1e-6);
    }

    // -----------------------------------------------------------------------
    // 6. Diagonal / combined speed
    // -----------------------------------------------------------------------

    @Test
    void diagonalSpeedBelowThreshold_isMotionless() {
        // vx=0.03, vy=0.03 → hypot ≈ 0.0424 < 0.05
        setLinearSpeed(0.03, 0.03);
        m_tracker.update();
        assertTrue(m_tracker.isMotionless());
    }

    @Test
    void diagonalSpeedAboveThreshold_isNotMotionless() {
        // vx=0.04, vy=0.04 → hypot ≈ 0.0566 > 0.05
        setLinearSpeed(0.04, 0.04);
        m_tracker.update();
        assertFalse(m_tracker.isMotionless());
    }
}
