package frc.robot.visutils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;


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
            DoubleSupplier hb,
            DoubleSupplier tid,
            Supplier<LimelightHelpers.PoseEstimate> mt1,
            Supplier<LimelightHelpers.PoseEstimate> mt2) {
        return new VisionHeartBeat(hb, tid, mt1, mt2, 1);
    }

    // -----------------------------------------------------------------------
    // Initial state
    // -----------------------------------------------------------------------

    @Test
    void initialState_allFalse() {
        VisionHeartBeat vhb = new VisionHeartBeat(
            () -> 0,
            () -> -1,
            TestVisionHeartBeat::invalidEstimate,
            TestVisionHeartBeat::invalidEstimate,
            5);
        assertFalse(vhb.isHeartbeating());
        assertFalse(vhb.hasSeenTid());
        assertFalse(vhb.hasSeenMt1Pose());
        assertFalse(vhb.hasSeenMt2Pose());
    }

    // -----------------------------------------------------------------------
    // Heartbeat
    // -----------------------------------------------------------------------

    @Test
    void heartbeat_notIncrementing_isFalse() {
        VisionHeartBeat vhb = build(
            () -> 42.0,
            () -> -1,
            TestVisionHeartBeat::invalidEstimate,
            TestVisionHeartBeat::invalidEstimate);
        vhb.update(); // first check — NaN → 42, not yet incrementing
        vhb.update(); // second check — 42 == 42
        assertFalse(vhb.isHeartbeating());
    }

    @Test
    void heartbeat_incrementing_isTrue() {
        AtomicLong counter = new AtomicLong(0);
        VisionHeartBeat vhb = build(
            counter::getAndIncrement,
            () -> -1,
            TestVisionHeartBeat::invalidEstimate,
            TestVisionHeartBeat::invalidEstimate);
        vhb.update(); // first check — NaN → 0
        vhb.update(); // second check — 0 → 1, incremented
        assertTrue(vhb.isHeartbeating());
    }

    @Test
    void heartbeat_stopsIncrementing_becomesFalse() {
        AtomicLong counter = new AtomicLong(0);
        AtomicReference<java.util.function.DoubleSupplier> supplier =
            new AtomicReference<>(counter::getAndIncrement);

        VisionHeartBeat vhb = build(
            () -> supplier.get().getAsDouble(),
            () -> -1,
            TestVisionHeartBeat::invalidEstimate,
            TestVisionHeartBeat::invalidEstimate);
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
    // hasSeenTid — latching behavior
    // -----------------------------------------------------------------------

    @Test
    void hasSeenTid_neverValid_isFalse() {
        AtomicLong hb = new AtomicLong(0);
        VisionHeartBeat vhb = build(hb::getAndIncrement, () -> -1.0,
            TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update(); // NaN → 0, not heartbeating
        vhb.update(); // heartbeating, tid = -1
        vhb.update(); // heartbeating, tid = -1
        assertFalse(vhb.hasSeenTid());
    }

    @Test
    void hasSeenTid_validOnce_latches() {
        AtomicLong hb = new AtomicLong(0);
        boolean[] useValid = {false};
        VisionHeartBeat vhb = build(hb::getAndIncrement,
            () -> useValid[0] ? 5.0 : -1.0,
            TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);

        vhb.update(); // not heartbeating
        vhb.update(); // heartbeating, tid = -1 → false
        assertFalse(vhb.hasSeenTid());

        useValid[0] = true;
        vhb.update(); // heartbeating, tid = 5 → true (latches)
        assertTrue(vhb.hasSeenTid());

        useValid[0] = false;
        vhb.update(); // heartbeating, already latched → still true
        assertTrue(vhb.hasSeenTid());
    }

    @Test
    void hasSeenTid_resetsOnHeartbeatLoss() {
        AtomicLong hb = new AtomicLong(0);
        AtomicReference<DoubleSupplier> hbSupplier = new AtomicReference<>(hb::getAndIncrement);

        VisionHeartBeat vhb = build(() -> hbSupplier.get().getAsDouble(), () -> 5.0,
            TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);

        vhb.update(); // NaN → 0, not heartbeating
        vhb.update(); // 0 → 1, heartbeating, tid = 5 → true
        assertTrue(vhb.hasSeenTid());

        // Freeze heartbeat at last-read value
        double lastRead = hb.get() - 1;
        hbSupplier.set(() -> lastRead);

        vhb.update(); // heartbeat frozen → not heartbeating → reset
        assertFalse(vhb.hasSeenTid());
    }

    // -----------------------------------------------------------------------
    // hasSeenMt1Pose — latching behavior
    // -----------------------------------------------------------------------

    @Test
    void hasSeenMt1Pose_neverValid_isFalse() {
        AtomicLong hb = new AtomicLong(0);
        VisionHeartBeat vhb = build(hb::getAndIncrement, () -> -1.0,
            TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update(); // not heartbeating
        vhb.update(); // heartbeating, mt1 = invalid
        assertFalse(vhb.hasSeenMt1Pose());
    }

    @Test
    void hasSeenMt1Pose_validOnce_latches() {
        AtomicLong hb = new AtomicLong(0);
        boolean[] useValid = {false};
        VisionHeartBeat vhb = build(hb::getAndIncrement, () -> -1.0,
            () -> useValid[0] ? validEstimate() : invalidEstimate(),
            TestVisionHeartBeat::invalidEstimate);

        vhb.update(); // not heartbeating
        vhb.update(); // heartbeating, mt1 = invalid → false
        assertFalse(vhb.hasSeenMt1Pose());

        useValid[0] = true;
        vhb.update(); // heartbeating, mt1 = valid → true (latches)
        assertTrue(vhb.hasSeenMt1Pose());

        useValid[0] = false;
        vhb.update(); // already latched → still true
        assertTrue(vhb.hasSeenMt1Pose());
    }

    @Test
    void hasSeenMt1Pose_resetsOnHeartbeatLoss() {
        AtomicLong hb = new AtomicLong(0);
        AtomicReference<DoubleSupplier> hbSupplier = new AtomicReference<>(hb::getAndIncrement);

        VisionHeartBeat vhb = build(() -> hbSupplier.get().getAsDouble(), () -> -1.0,
            TestVisionHeartBeat::validEstimate, TestVisionHeartBeat::invalidEstimate);

        vhb.update(); // not heartbeating
        vhb.update(); // heartbeating, mt1 = valid → true
        assertTrue(vhb.hasSeenMt1Pose());

        double lastRead = hb.get() - 1;
        hbSupplier.set(() -> lastRead);

        vhb.update(); // heartbeat frozen → reset
        assertFalse(vhb.hasSeenMt1Pose());
    }

    // -----------------------------------------------------------------------
    // hasSeenMt2Pose — latching behavior
    // -----------------------------------------------------------------------

    @Test
    void hasSeenMt2Pose_neverValid_isFalse() {
        AtomicLong hb = new AtomicLong(0);
        VisionHeartBeat vhb = build(hb::getAndIncrement, () -> -1.0,
            TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::invalidEstimate);
        vhb.update(); // not heartbeating
        vhb.update(); // heartbeating, mt2 = invalid
        assertFalse(vhb.hasSeenMt2Pose());
    }

    @Test
    void hasSeenMt2Pose_validOnce_latches() {
        AtomicLong hb = new AtomicLong(0);
        boolean[] useValid = {false};
        VisionHeartBeat vhb = build(hb::getAndIncrement, () -> -1.0,
            TestVisionHeartBeat::invalidEstimate,
            () -> useValid[0] ? validEstimate() : invalidEstimate());

        vhb.update(); // not heartbeating
        vhb.update(); // heartbeating, mt2 = invalid → false
        assertFalse(vhb.hasSeenMt2Pose());

        useValid[0] = true;
        vhb.update(); // heartbeating, mt2 = valid → true (latches)
        assertTrue(vhb.hasSeenMt2Pose());

        useValid[0] = false;
        vhb.update(); // already latched → still true
        assertTrue(vhb.hasSeenMt2Pose());
    }

    @Test
    void hasSeenMt2Pose_resetsOnHeartbeatLoss() {
        AtomicLong hb = new AtomicLong(0);
        AtomicReference<DoubleSupplier> hbSupplier = new AtomicReference<>(hb::getAndIncrement);

        VisionHeartBeat vhb = build(() -> hbSupplier.get().getAsDouble(), () -> -1.0,
            TestVisionHeartBeat::invalidEstimate, TestVisionHeartBeat::validEstimate);

        vhb.update(); // not heartbeating
        vhb.update(); // heartbeating, mt2 = valid → true
        assertTrue(vhb.hasSeenMt2Pose());

        double lastRead = hb.get() - 1;
        hbSupplier.set(() -> lastRead);

        vhb.update(); // heartbeat frozen → reset
        assertFalse(vhb.hasSeenMt2Pose());
    }

    // -----------------------------------------------------------------------
    // Interval skipping
    // -----------------------------------------------------------------------

    @Test
    void cachedValues_returnedBetweenChecks() {
        AtomicLong counter = new AtomicLong(0);
        AtomicReference<DoubleSupplier> supplier =
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
