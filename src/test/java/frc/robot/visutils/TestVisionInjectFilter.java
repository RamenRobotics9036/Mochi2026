package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.*;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

/**
 * Unit tests for {@link VisionInjectFilter}.
 *
 * <p>The filter has two independent rejection criteria:
 * <ol>
 *   <li><b>Timestamp window</b> – measurements whose (converted) timestamp falls within
 *       [resetTime − 2 s, resetTime + 0 s] are ignored.</li>
 *   <li><b>Distance</b> – measurements whose pose is more than 5 m from the current
 *       robot pose (or null) are ignored.</li>
 * </ol>
 *
 * <p>{@code Utils.fpgaToCurrentTime} is mocked so we can supply deterministic
 * converted timestamps without depending on real FPGA ↔ wall-clock offsets.
 */
class TestVisionInjectFilter {

    @BeforeAll
    static void initHal() {
        HAL.initialize(500, 0);
    }

    private VisionInjectFilter m_filter;

    /** Two poses that are close together (< 5 m apart). */
    private static final Pose2d POSE_A = new Pose2d(1.0, 1.0, Rotation2d.kZero);
    private static final Pose2d POSE_B = new Pose2d(2.0, 2.0, Rotation2d.kZero);

    /** A pose that is far from POSE_A (> 5 m). */
    private static final Pose2d POSE_FAR = new Pose2d(100.0, 100.0, Rotation2d.kZero);

    @BeforeEach
    void setUp() {
        m_filter = new VisionInjectFilter();
    }

    // -----------------------------------------------------------------------
    // 1. Startup behavior – no reset recorded
    // -----------------------------------------------------------------------

    @Test
    void noResetRecorded_shouldNotIgnore() {
        // Before any pose reset, the timestamp filter is disabled (sentinel value).
        // Close poses → distance check also passes.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(10.0)).thenReturn(10.0);
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 10.0));
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
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 9.0),
                "Measurement at 9.0 should be ignored (inside [8.0, 10.0])");
        }
    }

    @Test
    void timestampAtWindowStart_shouldIgnore() {
        // Exactly at the start boundary of the ignore window.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(8.0)).thenReturn(8.0);
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 8.0),
                "Measurement exactly at window start (8.0) should be ignored");
        }
    }

    @Test
    void timestampAtWindowEnd_shouldIgnore() {
        // Exactly at the end boundary of the ignore window (resetTime + 0.0).
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(10.0)).thenReturn(10.0);
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 10.0),
                "Measurement exactly at window end (10.0) should be ignored");
        }
    }

    @Test
    void timestampBeforeWindow_shouldNotIgnore() {
        // Measurement at 7.9 is before the ignore window [8.0, 10.0].
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(7.9)).thenReturn(7.9);
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 7.9),
                "Measurement at 7.9 should NOT be ignored (before window)");
        }
    }

    @Test
    void timestampAfterWindow_shouldNotIgnore() {
        // Measurement at 10.1 is after the ignore window [8.0, 10.0].
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(10.1)).thenReturn(10.1);
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 10.1),
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
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 20.0),
                "Should use converted timestamp (9.0, inside window) "
                + "not raw FPGA (20.0, outside window)");

            // Reverse: raw FPGA = 9.0 (INSIDE window → would be ignored if raw were used)
            // but fpgaToCurrentTime maps it to 20.0 (AFTER window → passes).
            mocked.when(() -> Utils.fpgaToCurrentTime(9.0)).thenReturn(20.0);
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 9.0),
                "Should use converted timestamp (20.0, outside window) "
                + "not raw FPGA (9.0, inside window)");
        }
    }

    // -----------------------------------------------------------------------
    // 3. Distance filtering
    // -----------------------------------------------------------------------

    @Test
    void closePoses_shouldNotIgnore() {
        // Both poses within 5 m, no reset → should pass.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 50.0));
        }
    }

    @Test
    void farAwayPoses_shouldIgnore() {
        // Distance between POSE_A(1,1) and POSE_FAR(100,100) >> 5 m.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertTrue(m_filter.shouldIgnore(POSE_FAR, POSE_A, 50.0),
                "Poses >5 m apart should be ignored");
        }
    }

    @Test
    void posesExactlyAtMaxDistance_shouldNotIgnore() {
        // Place poses exactly 5 m apart (should NOT be ignored; >5 is the threshold).
        Pose2d origin = new Pose2d(0, 0, Rotation2d.kZero);
        Pose2d atFive = new Pose2d(5.0, 0, Rotation2d.kZero);

        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertFalse(m_filter.shouldIgnore(atFive, origin, 50.0),
                "Poses exactly 5 m apart should NOT be ignored (threshold is >5)");
        }
    }

    @Test
    void posesJustOverMaxDistance_shouldIgnore() {
        Pose2d origin = new Pose2d(0, 0, Rotation2d.kZero);
        Pose2d justOver = new Pose2d(5.001, 0, Rotation2d.kZero);

        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertTrue(m_filter.shouldIgnore(justOver, origin, 50.0),
                "Poses just over 5 m should be ignored");
        }
    }

    @Test
    void nullVisionPose_shouldIgnore() {
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertTrue(m_filter.shouldIgnore(null, POSE_A, 50.0),
                "Null vision pose should be ignored");
        }
    }

    @Test
    void nullCurrentPose_shouldIgnore() {
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertTrue(m_filter.shouldIgnore(POSE_A, null, 50.0),
                "Null current pose should be ignored");
        }
    }

    @Test
    void bothPosesNull_shouldIgnore() {
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertTrue(m_filter.shouldIgnore(null, null, 50.0),
                "Both poses null should be ignored");
        }
    }

    // -----------------------------------------------------------------------
    // 4. Combined / interaction tests
    // -----------------------------------------------------------------------

    @Test
    void timestampInsideWindow_andClosePoses_shouldIgnore() {
        // Timestamp passes the time filter (ignored) but poses are close.
        // Time filter alone should cause rejection.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(9.0)).thenReturn(9.0);
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 9.0));
        }
    }

    @Test
    void timestampOutsideWindow_andFarPoses_shouldIgnore() {
        // Timestamp is fine but poses are far apart → should still be ignored.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(20.0)).thenReturn(20.0);
            assertTrue(m_filter.shouldIgnore(POSE_FAR, POSE_A, 20.0),
                "Far poses should be ignored even when timestamp is fine");
        }
    }

    @Test
    void timestampOutsideWindow_andClosePoses_shouldNotIgnore() {
        // Both checks pass → measurement is valid.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            m_filter.recordPoseReset(10.0);

            mocked.when(() -> Utils.fpgaToCurrentTime(20.0)).thenReturn(20.0);
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 20.0),
                "Valid timestamp + close poses → should NOT be ignored");
        }
    }

    // -----------------------------------------------------------------------
    // 5. Multiple resets
    // -----------------------------------------------------------------------

    @Test
    void secondReset_updatesWindow() {
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            // First reset at t = 10
            m_filter.recordPoseReset(10.0);

            // Measurement at t = 9 is inside first window [8, 10]
            mocked.when(() -> Utils.fpgaToCurrentTime(9.0)).thenReturn(9.0);
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 9.0),
                "Should be ignored under first reset window");

            // Second reset at t = 20 → new window is [18, 20]
            m_filter.recordPoseReset(20.0);

            // t = 9 is now outside the new window [18, 20]
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 9.0),
                "t=9 should now pass under the second reset window [18, 20]");

            // t = 19 is inside the new window
            mocked.when(() -> Utils.fpgaToCurrentTime(19.0)).thenReturn(19.0);
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 19.0),
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
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 4.9),
                "t=4.9 is before the window [5.0, 7.0]");

            // t = 5.0 is at the start of the window
            mocked.when(() -> Utils.fpgaToCurrentTime(5.0)).thenReturn(5.0);
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 5.0),
                "t=5.0 is at window start [5.0, 7.0]");

            // t = 6.5 is inside the window
            mocked.when(() -> Utils.fpgaToCurrentTime(6.5)).thenReturn(6.5);
            assertTrue(m_filter.shouldIgnore(POSE_A, POSE_B, 6.5),
                "t=6.5 is inside window [5.0, 7.0]");

            // t = 7.5 is outside the window
            mocked.when(() -> Utils.fpgaToCurrentTime(7.5)).thenReturn(7.5);
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_B, 7.5),
                "t=7.5 is after the window [5.0, 7.0]");
        }
    }

    // -----------------------------------------------------------------------
    // 6. Distance edge cases with rotation
    // -----------------------------------------------------------------------

    @Test
    void distanceChecksTranslationOnly_notRotation() {
        // Same position but different rotations → distance is 0, should not ignore.
        Pose2d poseRotated = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(180));

        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertFalse(m_filter.shouldIgnore(POSE_A, poseRotated, 50.0),
                "Same translation with different rotation should NOT be ignored");
        }
    }

    @Test
    void sameExactPose_shouldNotIgnore() {
        // Identical poses → distance is 0.
        try (MockedStatic<Utils> mocked = Mockito.mockStatic(Utils.class)) {
            mocked.when(() -> Utils.fpgaToCurrentTime(50.0)).thenReturn(50.0);
            assertFalse(m_filter.shouldIgnore(POSE_A, POSE_A, 50.0),
                "Identical poses should NOT be ignored");
        }
    }
}
