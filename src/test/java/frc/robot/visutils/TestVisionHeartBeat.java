package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.*;

class TestVisionHeartBeat {

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    private static LimelightHelpers.PoseEstimate validEstimate() {
        LimelightHelpers.RawFiducial rf =
            new LimelightHelpers.RawFiducial(1, 0, 0, 0, 1.0, 1.0, 0.1);
        return new LimelightHelpers.PoseEstimate(
            new Pose2d(), 0, 0, 1, 0, 1.0, 0,
            new LimelightHelpers.RawFiducial[]{rf}, false);
    }

    private static LimelightHelpers.PoseEstimate invalidEstimate() {
        return new LimelightHelpers.PoseEstimate(); // rawFiducials = empty array
    }

    /** Builds a VisionHeartBeat with checkInterval=1 so every update() triggers a real check. */
    private static VisionHeartBeat build(
            java.util.function.DoubleSupplier hb,
            java.util.function.DoubleSupplier tid,
            java.util.function.Supplier<LimelightHelpers.PoseEstimate> mt1,
            java.util.function.Supplier<LimelightHelpers.PoseEstimate> mt2) {
        return new VisionHeartBeat(hb, tid, mt1, mt2, 1);
    }

    // -----------------------------------------------------------------------
    // Initial state
    // -----------------------------------------------------------------------

    @Test
    void initialState_allFalse() {
        VisionHeartBeat vhb = new VisionHeartBeat(
            () -> 0, () -> -1, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate, 5);
        assertFalse(vhb.isHeartbeating());
        assertFalse(vhb.hasTid());
        assertFalse(vhb.hasMt1Pose());
        assertFalse(vhb.hasMt2Pose());
    }

    // -----------------------------------------------------------------------
    // Heartbeat
    // -----------------------------------------------------------------------

    @Test
    void heartbeat_notIncrementing_isFalse() {
        VisionHeartBeat vhb = build(() -> 42.0, () -> -1, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update(); // first check — NaN → 42, not yet incrementing
        vhb.update(); // second check — 42 == 42
        assertFalse(vhb.isHeartbeating());
    }

    @Test
    void heartbeat_incrementing_isTrue() {
        AtomicLong counter = new AtomicLong(0);
        VisionHeartBeat vhb = build(counter::getAndIncrement, () -> -1, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update(); // first check — NaN → 0
        vhb.update(); // second check — 0 → 1, incremented
        assertTrue(vhb.isHeartbeating());
    }

    @Test
    void heartbeat_stopsIncrementing_becomesFalse() {
        AtomicLong counter = new AtomicLong(0);
        AtomicReference<java.util.function.DoubleSupplier> supplier =
            new AtomicReference<>(counter::getAndIncrement);

        VisionHeartBeat vhb = build(() -> supplier.get().getAsDouble(), () -> -1, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update(); // NaN → 0
        vhb.update(); // 0 → 1, incrementing
        assertTrue(vhb.isHeartbeating());

        // Freeze at the last value that was actually read (counter is now 2, last read was 1)
        double lastRead = counter.get() - 1;
        supplier.set(() -> lastRead);

        vhb.update(); // lastRead == m_lastHeartbeat (same value)
        assertFalse(vhb.isHeartbeating());
    }

    // -----------------------------------------------------------------------
    // Tid
    // -----------------------------------------------------------------------

    @Test
    void tid_negativeOne_isFalse() {
        VisionHeartBeat vhb = build(() -> 0, () -> -1, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update();
        assertFalse(vhb.hasTid());
    }

    @Test
    void tid_validId_isTrue() {
        VisionHeartBeat vhb = build(() -> 0, () -> 5.0, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update();
        assertTrue(vhb.hasTid());
    }

    // -----------------------------------------------------------------------
    // MegaTag1 pose
    // -----------------------------------------------------------------------

    @Test
    void mt1_invalidEstimate_isFalse() {
        VisionHeartBeat vhb = build(() -> 0, () -> -1, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update();
        assertFalse(vhb.hasMt1Pose());
    }

    @Test
    void mt1_validEstimate_isTrue() {
        VisionHeartBeat vhb = build(() -> 0, () -> -1, TestVisionHeartBeat::validEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update();
        assertTrue(vhb.hasMt1Pose());
    }

    // -----------------------------------------------------------------------
    // MegaTag2 pose
    // -----------------------------------------------------------------------

    @Test
    void mt2_invalidEstimate_isFalse() {
        VisionHeartBeat vhb = build(() -> 0, () -> -1, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update();
        assertFalse(vhb.hasMt2Pose());
    }

    @Test
    void mt2_validEstimate_isTrue() {
        VisionHeartBeat vhb = build(() -> 0, () -> -1, TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::validEstimate);
        vhb.update();
        assertTrue(vhb.hasMt2Pose());
    }

    // -----------------------------------------------------------------------
    // Interval skipping
    // -----------------------------------------------------------------------

    @Test
    void cachedValues_returnedBetweenChecks() {
        AtomicLong counter = new AtomicLong(0);
        AtomicReference<java.util.function.DoubleSupplier> supplier =
            new AtomicReference<>(() -> 99.0); // frozen — not incrementing

        VisionHeartBeat vhb = new VisionHeartBeat(
            () -> supplier.get().getAsDouble(), () -> -1,
            TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate, 3);

        // cycle 1 — triggers first real check (NaN → 99, not incrementing yet)
        vhb.update();
        assertFalse(vhb.isHeartbeating());

        // switch to incrementing
        supplier.set(counter::getAndIncrement);

        // cycles 2 and 3 — skipped, cached false still returned
        vhb.update();
        assertFalse(vhb.isHeartbeating());
        vhb.update();
        assertFalse(vhb.isHeartbeating());

        // cycle 4 — triggers second real check, sees increment
        vhb.update();
        assertTrue(vhb.isHeartbeating());
    }
}
