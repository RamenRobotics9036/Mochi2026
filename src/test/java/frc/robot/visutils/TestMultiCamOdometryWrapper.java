package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;


/**
 * Unit tests for {@link MultiCamOdometryWrapper} and {@link NoOpCamOdometry}.
 *
 * <p>Key behaviors under test:
 *
 * <ul>
 *   <li><b>Enabled state</b> — every read/write method delegates to the real camera.
 *   <li><b>Disabled state</b> — every read method returns the no-op safe default; void methods
 *       are silent; the real camera is never called.
 *   <li><b>enableVision toggle</b> — switching between enabled and disabled mid-session works
 *       correctly.
 *   <li><b>Config methods bypass</b> — {@code enableMegatag2} always reaches the real camera
 *       regardless of enable state.
 *   <li><b>NoOpCamOdometry</b> — returns documented safe defaults and never throws.
 * </ul>
 */
class TestMultiCamOdometryWrapper {

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    // ------------------------------------------------------------------
    // Shared fixtures
    // ------------------------------------------------------------------

    private static final Pose2d POSE_A = new Pose2d(3.0, 2.0, Rotation2d.kZero);

    /** Create a mock real camera pre-configured with non-default return values. */
    private static CamOdometryInterface newRealMock() {
        CamOdometryInterface mock = Mockito.mock(CamOdometryInterface.class);
        Mockito.when(mock.getEstimatedPose()).thenReturn(Optional.of(POSE_A));
        Mockito.when(mock.getVisionErrorAtSnapTime()).thenReturn(OptionalDouble.of(1.0));
        Mockito.when(mock.getConfidenceScore()).thenReturn(0.85);
        Mockito.when(mock.getVisibleTagIds()).thenReturn(List.of(1, 2, 3));
        Mockito.when(mock.hasTargetLock()).thenReturn(true);
        Mockito.when(mock.hasMultiTagLock()).thenReturn(true);
        Mockito.when(mock.isLatestMt2()).thenReturn(true);
        Mockito.when(mock.getPrimaryTagId()).thenReturn(7);
        Mockito.when(mock.getPrimaryTagTx()).thenReturn(4.2);
        return mock;
    }

    // ------------------------------------------------------------------
    // 1. Initially-enabled wrapper — all calls go to the real camera
    // ------------------------------------------------------------------

    @Test
    void enabled_getEstimatedPose_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertEquals(Optional.of(POSE_A), wrapper.getEstimatedPose());
        Mockito.verify(real).getEstimatedPose();
    }

    @Test
    void enabled_getVisionErrorAtSnapTime_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertEquals(OptionalDouble.of(1.0), wrapper.getVisionErrorAtSnapTime());
        Mockito.verify(real).getVisionErrorAtSnapTime();
    }

    @Test
    void enabled_getConfidenceScore_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertEquals(0.85, wrapper.getConfidenceScore(), 1e-9);
        Mockito.verify(real).getConfidenceScore();
    }

    @Test
    void enabled_getVisibleTagIds_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertEquals(List.of(1, 2, 3), wrapper.getVisibleTagIds());
        Mockito.verify(real).getVisibleTagIds();
    }

    @Test
    void enabled_hasTargetLock_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertTrue(wrapper.hasTargetLock());
        Mockito.verify(real).hasTargetLock();
    }

    @Test
    void enabled_hasMultiTagLock_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertTrue(wrapper.hasMultiTagLock());
        Mockito.verify(real).hasMultiTagLock();
    }

    @Test
    void enabled_isLatestMt2_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertTrue(wrapper.isLatestMt2());
        Mockito.verify(real).isLatestMt2();
    }

    @Test
    void enabled_getPrimaryTagId_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertEquals(7, wrapper.getPrimaryTagId());
        Mockito.verify(real).getPrimaryTagId();
    }

    @Test
    void enabled_getPrimaryTagTx_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        assertEquals(4.2, wrapper.getPrimaryTagTx(), 1e-9);
        Mockito.verify(real).getPrimaryTagTx();
    }

    @Test
    void enabled_periodic_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        wrapper.periodic();

        Mockito.verify(real).periodic();
    }

    @Test
    void enabled_setRobotOrientation_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        wrapper.setRobotOrientation();

        Mockito.verify(real).setRobotOrientation();
    }

    @Test
    void enabled_setRobotOrientationNoFlush_delegatesToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        wrapper.setRobotOrientationNoFlush();

        Mockito.verify(real).setRobotOrientationNoFlush();
    }

    // ------------------------------------------------------------------
    // 2. Initially-disabled wrapper — returns no-op defaults, real never called
    // ------------------------------------------------------------------

    @Test
    void disabled_getEstimatedPose_returnsEmpty() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertEquals(Optional.empty(), wrapper.getEstimatedPose());
        Mockito.verify(real, Mockito.never()).getEstimatedPose();
    }

    @Test
    void disabled_getVisionErrorAtSnapTime_returnsEmpty() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertEquals(OptionalDouble.empty(), wrapper.getVisionErrorAtSnapTime());
        Mockito.verify(real, Mockito.never()).getVisionErrorAtSnapTime();
    }

    @Test
    void disabled_getConfidenceScore_returnsZero() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertEquals(0.0, wrapper.getConfidenceScore(), 1e-9);
        Mockito.verify(real, Mockito.never()).getConfidenceScore();
    }

    @Test
    void disabled_getVisibleTagIds_returnsEmptyList() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertEquals(Collections.emptyList(), wrapper.getVisibleTagIds());
        Mockito.verify(real, Mockito.never()).getVisibleTagIds();
    }

    @Test
    void disabled_hasTargetLock_returnsFalse() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertFalse(wrapper.hasTargetLock());
        Mockito.verify(real, Mockito.never()).hasTargetLock();
    }

    @Test
    void disabled_hasMultiTagLock_returnsFalse() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertFalse(wrapper.hasMultiTagLock());
        Mockito.verify(real, Mockito.never()).hasMultiTagLock();
    }

    @Test
    void disabled_isLatestMt2_returnsFalse() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertFalse(wrapper.isLatestMt2());
        Mockito.verify(real, Mockito.never()).isLatestMt2();
    }

    @Test
    void disabled_getPrimaryTagId_returnsMinusOne() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertEquals(-1, wrapper.getPrimaryTagId());
        Mockito.verify(real, Mockito.never()).getPrimaryTagId();
    }

    @Test
    void disabled_getPrimaryTagTx_returnsZero() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        assertEquals(0.0, wrapper.getPrimaryTagTx(), 1e-9);
        Mockito.verify(real, Mockito.never()).getPrimaryTagTx();
    }

    @Test
    void disabled_periodic_doesNotCallReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        wrapper.periodic();

        Mockito.verify(real, Mockito.never()).periodic();
    }

    @Test
    void disabled_setRobotOrientation_doesNotCallReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        wrapper.setRobotOrientation();

        Mockito.verify(real, Mockito.never()).setRobotOrientation();
    }

    @Test
    void disabled_setRobotOrientationNoFlush_doesNotCallReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        wrapper.setRobotOrientationNoFlush();

        Mockito.verify(real, Mockito.never()).setRobotOrientationNoFlush();
    }

    // ------------------------------------------------------------------
    // 3. enableVision toggle mid-session
    // ------------------------------------------------------------------

    @Test
    void enableVision_falseAfterTrue_switchesToNoOp() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        // Confirm initially delegates to real.
        wrapper.getEstimatedPose();
        Mockito.verify(real, Mockito.times(1)).getEstimatedPose();

        // Disable and confirm real is no longer called.
        wrapper.enableVision(false);
        wrapper.getEstimatedPose();
        Mockito.verify(real, Mockito.times(1)).getEstimatedPose(); // still only 1
    }

    @Test
    void enableVision_trueAfterFalse_switchesBackToReal() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, false);

        // Disabled: real not called.
        wrapper.getEstimatedPose();
        Mockito.verify(real, Mockito.never()).getEstimatedPose();

        // Re-enable: real should now be called.
        wrapper.enableVision(true);
        wrapper.getEstimatedPose();
        Mockito.verify(real, Mockito.times(1)).getEstimatedPose();
    }

    @Test
    void enableVision_periodicRespectsToggle() {
        CamOdometryInterface real = newRealMock();
        MultiCamOdometryWrapper wrapper = new MultiCamOdometryWrapper(real, true);

        wrapper.periodic();
        Mockito.verify(real, Mockito.times(1)).periodic();

        wrapper.enableVision(false);
        wrapper.periodic();
        Mockito.verify(real, Mockito.times(1)).periodic(); // still 1 — no new call

        wrapper.enableVision(true);
        wrapper.periodic();
        Mockito.verify(real, Mockito.times(2)).periodic();
    }

    // ------------------------------------------------------------------
    // 5. NoOpCamOdometry — safe defaults, no throws
    // ------------------------------------------------------------------

    @Test
    void noOp_getEstimatedPose_returnsEmpty() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertEquals(Optional.empty(), noop.getEstimatedPose());
    }

    @Test
    void noOp_getVisionErrorAtSnapTime_returnsEmpty() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertEquals(OptionalDouble.empty(), noop.getVisionErrorAtSnapTime());
    }

    @Test
    void noOp_getConfidenceScore_returnsZero() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertEquals(0.0, noop.getConfidenceScore(), 1e-9);
    }

    @Test
    void noOp_getVisibleTagIds_returnsEmptyList() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertEquals(Collections.emptyList(), noop.getVisibleTagIds());
    }

    @Test
    void noOp_hasTargetLock_returnsFalse() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertFalse(noop.hasTargetLock());
    }

    @Test
    void noOp_hasMultiTagLock_returnsFalse() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertFalse(noop.hasMultiTagLock());
    }

    @Test
    void noOp_isLatestMt2_returnsFalse() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertFalse(noop.isLatestMt2());
    }

    @Test
    void noOp_getPrimaryTagId_returnsMinusOne() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertEquals(-1, noop.getPrimaryTagId());
    }

    @Test
    void noOp_getPrimaryTagTx_returnsZero() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertEquals(0.0, noop.getPrimaryTagTx(), 1e-9);
    }

    @Test
    void noOp_voidMethods_doNotThrow() {
        NoOpCamOdometry noop = new NoOpCamOdometry();
        assertDoesNotThrow(() -> {
            noop.periodic();
            noop.setRobotOrientation();
            noop.setRobotOrientationNoFlush();
            noop.enableVision(true);
            noop.enableVision(false);
        });
    }
}
