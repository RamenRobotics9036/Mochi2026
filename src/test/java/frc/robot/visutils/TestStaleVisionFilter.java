package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

/**
 * Unit tests for {@link StaleVisionFilter}.
 *
 * <p>The filter rejects measurements whose (converted) timestamp falls within
 * [resetTime − 2 s, resetTime + 0 s].
 *
 * <p>{@code Utils.fpgaToCurrentTime} is mocked so we can supply deterministic
 * converted timestamps without depending on real FPGA ↔ wall-clock offsets.
 */
class TestStaleVisionFilter {

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    private StaleVisionFilter m_filter;

    @BeforeEach
    void setUp() {
        m_filter = new StaleVisionFilter();
    }

    // -----------------------------------------------------------------------
    // 1. Startup behavior – no reset recorded
    // -----------------------------------------------------------------------

    @Test
    void noResetRecorded_shouldNotIgnore() {
        // Before any pose reset, the timestamp filter is disabled (sentinel value).
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(10.0)).thenReturn(10.0);
            assertFalse(m_filter.shouldIgnore(10.0));
        }
    }

    // -----------------------------------------------------------------------
    // 2. Timestamp-window filtering
    // -----------------------------------------------------------------------

    @Test
    void timestampInsideWindow_shouldIgnore() {
        // Reset at t = 10.0 → ignore window is [8.0, 10.0].
        // Measurement with converted timestamp 9.0 falls inside the window.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(9.0)).thenReturn(9.0);
            assertTrue(m_filter.shouldIgnore(9.0),
                "Measurement at 9.0 should be ignored (inside [8.0, 10.0])");
        }
    }

    @Test
    void timestampAtWindowStart_shouldIgnore() {
        // Exactly at the start boundary of the ignore window.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(8.0)).thenReturn(8.0);
            assertTrue(m_filter.shouldIgnore(8.0),
                "Measurement exactly at window start (8.0) should be ignored");
        }
    }

    @Test
    void timestampAtWindowEnd_shouldIgnore() {
        // Exactly at the end boundary of the ignore window (resetTime + 0.0).
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(10.0)).thenReturn(10.0);
            assertTrue(m_filter.shouldIgnore(10.0),
                "Measurement exactly at window end (10.0) should be ignored");
        }
    }

    @Test
    void timestampBeforeWindow_shouldNotIgnore() {
        // Measurement at 7.9 is before the ignore window [8.0, 10.0].
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(7.9)).thenReturn(7.9);
            assertFalse(m_filter.shouldIgnore(7.9),
                "Measurement at 7.9 should NOT be ignored (before window)");
        }
    }

    @Test
    void timestampAfterWindow_shouldNotIgnore() {
        // Measurement at 10.1 is after the ignore window [8.0, 10.0].
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(10.1)).thenReturn(10.1);
            assertFalse(m_filter.shouldIgnore(10.1),
                "Measurement at 10.1 should NOT be ignored (after window)");
        }
    }

    @Test
    void fpgaConversion_usesConvertedTimestamp() {
        // Verify the filter uses the CONVERTED timestamp, not the raw FPGA one.
        // Reset at 10.0 → window is [8.0, 10.0].
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            // Raw FPGA = 20.0 (AFTER window → would pass if raw were used)
            // but fpgaToCurrentTime maps it to 9.0 (INSIDE window → ignored).
            mocked.when(() -> Utils.fpgaToCurrentTime(20.0)).thenReturn(9.0);
            assertTrue(m_filter.shouldIgnore(20.0),
                "Should use converted timestamp (9.0, inside window) "
                + "not raw FPGA (20.0, outside window)");

            // Reverse: raw FPGA = 9.0 (INSIDE window → would be ignored if raw were used)
            // but fpgaToCurrentTime maps it to 20.0 (AFTER window → passes).
            mocked.when(() -> Utils.fpgaToCurrentTime(9.0)).thenReturn(20.0);
            assertFalse(m_filter.shouldIgnore(9.0),
                "Should use converted timestamp (20.0, outside window) "
                + "not raw FPGA (9.0, inside window)");
        }
    }

    // -----------------------------------------------------------------------
    // 3. Combined / interaction tests
    // -----------------------------------------------------------------------

    @Test
    void timestampInsideWindow_shouldIgnore_afterReset() {
        // Timestamp falls inside the ignore window → rejected.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(9.0)).thenReturn(9.0);
            assertTrue(m_filter.shouldIgnore(9.0));
        }
    }

    @Test
    void timestampOutsideWindow_shouldNotIgnore() {
        // Timestamp is outside the window → passes.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(20.0)).thenReturn(20.0);
            assertFalse(m_filter.shouldIgnore(20.0),
                "Valid timestamp outside reset window → should NOT be ignored");
        }
    }

    // -----------------------------------------------------------------------
    // 4. Multiple resets
    // -----------------------------------------------------------------------

    @Test
    void secondReset_updatesWindow() {
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            // First reset at t = 10
            m_filter.recordPoseReset(10.0);

            // Measurement at t = 9 is inside first window [8, 10]
            mocked.when(() -> Utils.fpgaToCurrentTime(9.0)).thenReturn(9.0);
            assertTrue(m_filter.shouldIgnore(9.0),
                "Should be ignored under first reset window");

            // Second reset at t = 20 → new window is [18, 20]
            m_filter.recordPoseReset(20.0);

            // t = 9 is now outside the new window [18, 20]
            assertFalse(m_filter.shouldIgnore(9.0),
                "t=9 should now pass under the second reset window [18, 20]");

            // t = 19 is inside the new window
            mocked.when(() -> Utils.fpgaToCurrentTime(19.0)).thenReturn(19.0);
            assertTrue(m_filter.shouldIgnore(19.0),
                "t=19 should be ignored under second reset window [18, 20]");
        }
    }

    @Test
    void multipleResetsInRapidSuccession_usesLastReset() {
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(5.0);
            m_filter.recordPoseReset(6.0);
            m_filter.recordPoseReset(7.0); // only this one should matter

            // Window is [5.0, 7.0] for the last reset.
            // t = 4.9 is outside the last window
            mocked.when(() -> Utils.fpgaToCurrentTime(4.9)).thenReturn(4.9);
            assertFalse(m_filter.shouldIgnore(4.9),
                "t=4.9 is before the window [5.0, 7.0]");

            // t = 5.0 is at the start of the window
            mocked.when(() -> Utils.fpgaToCurrentTime(5.0)).thenReturn(5.0);
            assertTrue(m_filter.shouldIgnore(5.0),
                "t=5.0 is at window start [5.0, 7.0]");

            // t = 6.5 is inside the window
            mocked.when(() -> Utils.fpgaToCurrentTime(6.5)).thenReturn(6.5);
            assertTrue(m_filter.shouldIgnore(6.5),
                "t=6.5 is inside window [5.0, 7.0]");

            // t = 7.5 is outside the window
            mocked.when(() -> Utils.fpgaToCurrentTime(7.5)).thenReturn(7.5);
            assertFalse(m_filter.shouldIgnore(7.5),
                "t=7.5 is after the window [5.0, 7.0]");
        }
    }
}
